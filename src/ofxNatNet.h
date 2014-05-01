#pragma once

#include "ofMain.h"

class ofxNatNet
{
	class InternalThread;
	friend class InternalThread;

public:
	typedef ofVec3f Marker;

	class RigidBody
	{
		friend class InternalThread;

	public:
		int id;
		ofMatrix4x4 matrix;
		vector<Marker> markers;

		float mean_marker_error;

		inline bool isActive() const { return _active; }
		OF_DEPRECATED_MSG("Use isActive insted.", bool active() const);

		const ofMatrix4x4& getMatrix() const { return matrix; }

	private:
		bool _active;
		ofVec3f raw_position;
	};

	ofxNatNet()
		: thread(NULL)
		, frame_number(0)
		, latency(0)
	{
	}
	~ofxNatNet() { dispose(); }

	void setup(string interface_name, string target_host,
			   string multicast_group = "239.255.42.99",
			   int command_port = 1510, int data_port = 1511);
	void update();

	void sendPing();

	bool isConnected();
	int getFrameNumber() { return frame_number; }
	float getLatency() { return latency; }

	float getDataRate();

	void setScale(float v);
	ofVec3f getScale();

	void setTransform(const ofMatrix4x4& m);
	const ofMatrix4x4& getTransform();

	void setDuplicatedPointRemovalDistance(float v);

	inline const size_t getNumMarker() { return markers.size(); }
	inline const Marker& getMarker(size_t index) { return markers[index]; }

	inline const size_t getNumFilterdMarker() { return filterd_markers.size(); }
	inline const Marker& getFilterdMarker(size_t index)
	{
		return filterd_markers[index];
	}

	inline const size_t getNumRigidBody() { return rigidbodies.size(); }
	inline const RigidBody& getRigidBodyAt(int index)
	{
		return *rigidbodies_arr[index];
	}

	inline const bool hasRigidBody(int id)
	{
		return rigidbodies.find(id) != rigidbodies.end();
	}
	inline const bool getRigidBody(int id, RigidBody& RB)
	{
		if (!hasRigidBody(id)) return false;
		RB = rigidbodies[id];
		return true;
	}

	void setBufferTime(float sec);
	int getBufferTime();

	void forceSetNatNetVersion(int v);

	void debugDraw();
	void debugDrawMarkers();

protected:
	InternalThread* thread;

	int frame_number;
	float latency;

	vector<Marker> filterd_markers;
	vector<Marker> markers;

	map<int, RigidBody> rigidbodies;
	vector<RigidBody*> rigidbodies_arr;

	void dispose();

private:
	ofxNatNet(const ofxNatNet&);
	ofxNatNet& operator=(const ofxNatNet&);
};

//

inline bool ofxNatNet::RigidBody::active() const { return isActive(); }
