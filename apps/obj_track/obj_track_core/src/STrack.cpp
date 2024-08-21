#include "../include/STrack.h"
#include <algorithm>

IdAllocator STrack::allocator_ = IdAllocator(201, 1.0);
STrack::STrack(std::vector<float> tlwh_, float score, int label)
{
	_tlwh.resize(4);
	_tlwh.assign(tlwh_.begin(), tlwh_.end());

	is_activated_ = false;
	track_id_ = 0;
	state_ = TrackState::New;
	frame_lost_ = 0;
	
	tlwh.resize(4);
	tlbr.resize(4);

	static_tlwh();
	static_tlbr();
	frame_id_ = 0;
	tracklet_len_ = 0;
	this->score_ = score;
	start_frame_ = 0;

	this->label_ = label;
	conf_ = std::min((float)0.99, score);
}

STrack::~STrack()
{ allocator_.InvalidateId(track_id_);
}

void STrack::Activate(byte_kalman::KalmanFilter &kalman_filter, int frame_id)
{
	this->kalman_filter_ = kalman_filter;
	// this->track_id_ = this->NextId();
	// this->track_id_ = alloter_.GetNewId();
	this->track_id_ = -1;

	std::vector<float> _tlwh_tmp(4);
	_tlwh_tmp[0] = this->_tlwh[0];
	_tlwh_tmp[1] = this->_tlwh[1];
	_tlwh_tmp[2] = this->_tlwh[2];
	_tlwh_tmp[3] = this->_tlwh[3];
	std::vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter_.Initiate(xyah_box);
	this->mean_ = mc.first;
	this->covariance_ = mc.second;

	static_tlwh();
	static_tlbr();

	this->tracklet_len_ = 0;
	this->state_ = TrackState::Tracked;
	if (frame_id == 1)
	{
		this->is_activated_ = true;
		this->track_id_ = allocator_.GetNewId();
	}
	//this->is_activated = true;
	this->frame_id_ = frame_id;
	this->start_frame_ = frame_id;
}

void STrack::ReActivate(STrack &new_track, int frame_id, bool new_id)
{
	std::vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter_.Update(this->mean_, this->covariance_, xyah_box);
	this->mean_ = mc.first;
	this->covariance_ = mc.second;

	this->_tlwh = new_track._tlwh;

	static_tlwh();
	static_tlbr();

	this->tracklet_len_ = 0;
	this->state_ = TrackState::Tracked;
	this->frame_lost_ = 0;
	this->is_activated_ = true;
	this->frame_id_ = frame_id;
	this->score_ = new_track.score_;
	this->label_ = new_track.label_;
	if (new_id)
		this->track_id_ = NextId();
	this->det_index_ = new_track.det_index_;
}

void STrack::Update(STrack &new_track, int frame_id)
{
	if (this->track_id_ == -1) this->track_id_ = allocator_.GetNewId();
	this->frame_id_ = frame_id;
	this->tracklet_len_++;

	std::vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];

	auto mc = this->kalman_filter_.Update(this->mean_, this->covariance_, xyah_box);
	this->mean_ = mc.first;
	this->covariance_ = mc.second;

	this->_tlwh = new_track._tlwh;

	static_tlwh();
	static_tlbr();

	this->state_ = TrackState::Tracked;
	this->is_activated_ = true;

	this->score_ = new_track.score_;
	this->label_ = new_track.label_;
	this->det_index_ = new_track.det_index_;
}

void STrack::static_tlwh()
{
	if (this->state_ == TrackState::New)
	{
		tlwh[0] = _tlwh[0];
		tlwh[1] = _tlwh[1];
		tlwh[2] = _tlwh[2];
		tlwh[3] = _tlwh[3];
		return;
	}

	tlwh[0] = mean_[0];
	tlwh[1] = mean_[1];
	tlwh[2] = mean_[2];
	tlwh[3] = mean_[3];

	tlwh[2] *= tlwh[3];
	tlwh[0] -= tlwh[2] / 2;
	tlwh[1] -= tlwh[3] / 2;
}

void STrack::static_tlbr()
{
	tlbr.clear();
	tlbr.assign(tlwh.begin(), tlwh.end());
	tlbr[2] += tlbr[0];
	tlbr[3] += tlbr[1];
}

std::vector<float> STrack::tlwh_to_xyah(std::vector<float> tlwh_tmp)
{
	std::vector<float> tlwh_output = tlwh_tmp;
	tlwh_output[0] += tlwh_output[2] / 2;
	tlwh_output[1] += tlwh_output[3] / 2;
	tlwh_output[2] /= tlwh_output[3];
	return tlwh_output;
}

std::vector<float> STrack::to_xyah()
{
	return tlwh_to_xyah(tlwh);
}

std::vector<float> STrack::tlbr_to_tlwh(std::vector<float> &tlbr)
{
	tlbr[2] -= tlbr[0];
	tlbr[3] -= tlbr[1];
	return tlbr;
}

void STrack::MarkLost()
{
	state_ = TrackState::Lost;
	frame_lost_ = 1;
}

void STrack::MarkRemoved()
{
	state_ = TrackState::Removed;
}

int STrack::NextId()
{
	static int _count = 0;
	_count++;
	return _count;
}

int STrack::EndFrame()
{
	return this->frame_id_;
}

void STrack::UpdateWithDet(float ps, float loss) {
	conf_ = (conf_ * ps * (1.0 - loss)) / 
			((conf_ * ps * (1.0 - loss)) + 
			(1.0 - conf_) * (1.0 - ps * (1.0 - loss)));
	conf_ = conf_ > 0.99 ? 0.99 : conf_;
}

void STrack::UpdateWithoutDet(float ps) {
	conf_ = (conf_ * (1.0 - ps)) /
			((conf_ * (1.0 - ps)) + ps * (1.0 - conf_));
	det_index_ = INT_MAX;
}

void STrack::MultiPredict(std::vector<STrack*> &stracks, byte_kalman::KalmanFilter &kalman_filter)
{
	for (int i = 0; i < stracks.size(); i++)
	{
		if (stracks[i]->state_ != TrackState::Tracked)
		{
			stracks[i]->mean_[7] = 0;
		}
		kalman_filter.Predict(stracks[i]->mean_, stracks[i]->covariance_);
		stracks[i]->static_tlwh();
		stracks[i]->static_tlbr();
	}
}