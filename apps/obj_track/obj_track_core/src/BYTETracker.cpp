#include "../include/BYTETracker.h"

#include <fstream>

BYTETracker::BYTETracker(int frame_rate, int track_buffer) {
	track_thresh_ = 0.5;
	high_thresh_ = 0.6;
	match_thresh_ = 0.8;
	sensor_conf_ = 0.7;
	output_thresh_ = 0.6;
	resolution_ratio_[0] = 1920.0;
	resolution_ratio_[1] = 1080.0;

	frame_id_ = 0;
	max_time_lost_ = int(frame_rate / 30.0 * track_buffer);
	// std::cout << "Init ByteTrack!" << std::endl;
}

BYTETracker::~BYTETracker() {}

bool BYTETracker::SetTrackThresh(float track_thresh) {
	if (frame_id_ != 0) return false;
	this->track_thresh_ = track_thresh;
	return true;
}

bool BYTETracker::SetHighThresh(float high_thresh) {
	if (frame_id_ != 0) return false;
	this->high_thresh_ = high_thresh;
	return true;
}

bool BYTETracker::SetMatchThresh(float match_thresh) {
	if (frame_id_ != 0) return false;
	this->match_thresh_ = match_thresh;
	return true;
}

bool BYTETracker::SetSensorConfidence(float sensor_conf) {
	if (frame_id_ != 0) return false;
	this->sensor_conf_ = sensor_conf;
	return true;
}

bool BYTETracker::SetOutputThresh(float output_thresh) {
	if (frame_id_ != 0) return false;
	this->output_thresh_ = output_thresh;
	return true;
}

bool BYTETracker::SetResolutionRatio(float width, float height) {
	if (frame_id_ != 0) return false;
	this->resolution_ratio_[0] = width;
	this->resolution_ratio_[1] = height;
	return true;
}

bool BYTETracker::SetWeightPosition(float weight_position) {
	if (frame_id_ != 0) return false;
	this->kalman_filter_.SetWeightPosition(weight_position);
	return true;
}

bool BYTETracker::SetWeightVelocity(float weight_velocity) {
	if (frame_id_ != 0) return false;
	this->kalman_filter_.SetWeightVelocity(weight_velocity);
	return true;
}

std::vector<STrack> BYTETracker::Update(const std::vector<Obj>& objects) {
	////////////////// Step 1: Get detections //////////////////
	this->frame_id_++;
	std::vector<STrack> activated_stracks;
	std::vector<STrack> refind_stracks;
	std::vector<STrack> removed_stracks;
	std::vector<STrack> lost_stracks;
	std::vector<STrack> detections;
	std::vector<STrack> detections_low;

	std::vector<STrack> detections_cp;
	std::vector<STrack> tracked_stracks_swap;
	std::vector<STrack> resa, resb;
	std::vector<STrack> output_stracks;

	std::vector<STrack*> unconfirmed;
	std::vector<STrack*> tracked_stracks;
	std::vector<STrack*> strack_pool;
	std::vector<STrack*> r_tracked_stracks;

	if (objects.size() > 0) {
		for (int i = 0; i < objects.size(); i++) {
			std::vector<float> tlbr_;
			tlbr_.resize(4);
			tlbr_[0] = objects[i].rect.x;
			tlbr_[1] = objects[i].rect.y;
			tlbr_[2] = objects[i].rect.x + objects[i].rect.width;
			tlbr_[3] = objects[i].rect.y + objects[i].rect.height;

			float score = objects[i].prob;
			int label = objects[i].label;

			STrack strack(STrack::tlbr_to_tlwh(tlbr_), score, label);
			strack.det_index_ = objects[i].index;
			if (score >= track_thresh_) {
				detections.push_back(strack);
			}
			else {
				detections_low.push_back(strack);
			}
		}
	}

	// Add newly detected tracklets to tracked_stracks
	for (int i = 0; i < this->tracked_stracks_.size(); i++) {
		if (!this->tracked_stracks_[i].is_activated_)
			unconfirmed.push_back(&this->tracked_stracks_[i]);
		else
			tracked_stracks.push_back(&this->tracked_stracks_[i]);
	}

	////////////////// Step 2: First association, with IoU //////////////////
	strack_pool = JointStracks(tracked_stracks, this->lost_stracks_);
	STrack::MultiPredict(strack_pool, this->kalman_filter_);

	std::vector<std::vector<float> > dists;
	int dist_size = 0, dist_size_size = 0;
	dists = IouDistance(strack_pool, detections, dist_size, dist_size_size);

	std::vector<std::vector<int> > matches;
	std::vector<int> u_track, u_detection;
	LinearAssignment(dists, dist_size, dist_size_size, match_thresh_, matches, u_track, u_detection);

	for (int i = 0; i < matches.size(); i++) {
		STrack *track = strack_pool[matches[i][0]];
		STrack *det = &detections[matches[i][1]];
		if (track->state_ == TrackState::Tracked) {
			track->Update(*det, this->frame_id_);
			track->UpdateWithDet(sensor_conf_, dists[matches[i][0]][matches[i][1]]);
			activated_stracks.push_back(*track);
		}
		else {
			track->ReActivate(*det, this->frame_id_, false);
			track->UpdateWithDet(sensor_conf_, dists[matches[i][0]][matches[i][1]]);
			refind_stracks.push_back(*track);
		}
	}

	////////////////// Step 3: Second association, using low score dets //////////////////
	for (int i = 0; i < u_detection.size(); i++) {
		detections_cp.push_back(detections[u_detection[i]]);
	}
	detections.clear();
	detections.assign(detections_low.begin(), detections_low.end());
	
	for (int i = 0; i < u_track.size(); i++) {
		if (strack_pool[u_track[i]]->state_ == TrackState::Tracked) {
			r_tracked_stracks.push_back(strack_pool[u_track[i]]);
		} else {
			strack_pool[u_track[i]]->frame_lost_++;
			// strack_pool[u_track[i]]->conf = 
		}
	}

	dists.clear();
	dists = IouDistance(r_tracked_stracks, detections, dist_size, dist_size_size);

	matches.clear();
	u_track.clear();
	u_detection.clear();
	LinearAssignment(dists, dist_size, dist_size_size, 0.5, matches, u_track, u_detection);

	for (int i = 0; i < matches.size(); i++) {
		STrack *track = r_tracked_stracks[matches[i][0]];
		STrack *det = &detections[matches[i][1]];
		if (track->state_ == TrackState::Tracked) {
			track->Update(*det, this->frame_id_);
			track->UpdateWithDet(sensor_conf_, dists[matches[i][0]][matches[i][1]]);
			activated_stracks.push_back(*track);
		}
		else {
			track->ReActivate(*det, this->frame_id_, false);
			track->UpdateWithDet(sensor_conf_, dists[matches[i][0]][matches[i][1]]);
			refind_stracks.push_back(*track);
		}
	}

	for (int i = 0; i < u_track.size(); i++) {
		STrack *track = r_tracked_stracks[u_track[i]];
		if (track->state_ != TrackState::Lost) {
			track->MarkLost();
			lost_stracks.push_back(*track);
		}
	}

	// Deal with unconfirmed tracks, usually tracks with only one beginning frame
	detections.clear();
	detections.assign(detections_cp.begin(), detections_cp.end());

	dists.clear();
	dists = IouDistance(unconfirmed, detections, dist_size, dist_size_size);

	matches.clear();
	std::vector<int> u_unconfirmed;
	u_detection.clear();
	LinearAssignment(dists, dist_size, dist_size_size, 0.7, matches, u_unconfirmed, u_detection);

	for (int i = 0; i < matches.size(); i++) {
		unconfirmed[matches[i][0]]->Update(detections[matches[i][1]], this->frame_id_);
		unconfirmed[matches[i][0]]->UpdateWithDet(sensor_conf_, dists[matches[i][0]][matches[i][1]]);
		activated_stracks.push_back(*unconfirmed[matches[i][0]]);
	}

	for (int i = 0; i < u_unconfirmed.size(); i++) {
		STrack *track = unconfirmed[u_unconfirmed[i]];
		track->MarkRemoved();
		removed_stracks.push_back(*track);
	}

	////////////////// Step 4: Init new stracks //////////////////
	for (int i = 0; i < u_detection.size(); i++) {
		STrack *track = &detections[u_detection[i]];
		if (track->score_ < this->high_thresh_)
			continue;
		track->Activate(this->kalman_filter_, this->frame_id_);
		activated_stracks.push_back(*track);
	}

	////////////////// Step 5: Update state //////////////////
	for (int i = 0; i < this->lost_stracks_.size(); i++) {
		if (this->frame_id_ - this->lost_stracks_[i].EndFrame() > this->max_time_lost_) {
			this->lost_stracks_[i].MarkRemoved();
			removed_stracks.push_back(this->lost_stracks_[i]);
		}
	}
	
	for (int i = 0; i < this->tracked_stracks_.size(); i++) {
		if (this->tracked_stracks_[i].state_ == TrackState::Tracked) {
			tracked_stracks_swap.push_back(this->tracked_stracks_[i]);
		}
	}
	this->tracked_stracks_.clear();
	this->tracked_stracks_.assign(tracked_stracks_swap.begin(), tracked_stracks_swap.end());

	this->tracked_stracks_ = JointStracks(this->tracked_stracks_, activated_stracks);
	this->tracked_stracks_ = JointStracks(this->tracked_stracks_, refind_stracks);

	//std::cout << activated_stracks.size() << std::endl;

	this->lost_stracks_ = SubStracks(this->lost_stracks_, this->tracked_stracks_);
	for (int i = 0; i < lost_stracks.size(); i++) {
		this->lost_stracks_.push_back(lost_stracks[i]);
	}

	this->lost_stracks_ = SubStracks(this->lost_stracks_, this->removed_stracks_);
	// 不再保存已删除的轨迹
	this->removed_stracks_.clear();
	for (int i = 0; i < removed_stracks.size(); i++) {
		this->removed_stracks_.push_back(removed_stracks[i]);
	}
	
	RemoveDuplicateStracks(resa, resb, this->tracked_stracks_, this->lost_stracks_);

	this->tracked_stracks_.clear();
	this->tracked_stracks_.assign(resa.begin(), resa.end());
	this->lost_stracks_.clear();
	this->lost_stracks_.assign(resb.begin(), resb.end());
	
	for (int i = 0; i < this->tracked_stracks_.size(); i++) {
		if (this->tracked_stracks_[i].is_activated_) {
			if (this->tracked_stracks_[i].tlwh[0] > 10 &&
					this->tracked_stracks_[i].tlwh[1] > 10 &&
					this->tracked_stracks_[i].tlwh[0] + this->tracked_stracks_[i].tlwh[2] < (this->resolution_ratio_[0] - 10) &&
					this->tracked_stracks_[i].tlwh[1] + this->tracked_stracks_[i].tlwh[3] < (this->resolution_ratio_[1] - 10))
				output_stracks.push_back(this->tracked_stracks_[i]);
		}
	}

	for (int i = 0; i < this->lost_stracks_.size(); i++) {
		this->lost_stracks_[i].UpdateWithoutDet(sensor_conf_);
		if (this->lost_stracks_[i].conf_ > output_thresh_) {
			if (this->lost_stracks_[i].tlwh[0] > 10 &&
					this->lost_stracks_[i].tlwh[1] > 10 &&
					this->lost_stracks_[i].tlwh[0] + this->lost_stracks_[i].tlwh[2] < (this->resolution_ratio_[0] - 10) &&
					this->lost_stracks_[i].tlwh[1] + this->lost_stracks_[i].tlwh[3] < (this->resolution_ratio_[1] - 10))
				output_stracks.push_back(this->lost_stracks_[i]);
		}
	}
	return output_stracks;
}