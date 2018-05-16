#pragma once

#include "ofMain.h"
#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/MulticastSocket.h>
#include <Poco/Net/NetworkInterface.h>
#include <Poco/Net/NetException.h>

class ofxNatNet
{
	struct InternalThread;
	friend struct InternalThread;

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
	
	class Skeleton
	{
	public:
		int id;
		vector<RigidBody> joints;
	};
    
    class RigidBodyDescription
    {
    public:
        string name;
        int id;
        int parent_id;
        ofVec3f offset;
        vector<string> marker_names;
    };
    
    class SkeletonDescription
    {
    public:
        string name;
        int id;
        vector<RigidBodyDescription> joints;
    };
    
    class MarkerSetDescription
    {
    public:
        string name;
        vector<string> marker_names;
    };

	ofxNatNet()
		: thread(NULL)
		, frame_number(0)
		, latency(0)
		, timeout(0.1)
	{
	}
	~ofxNatNet() { dispose(); }

	void setup(string interface_name, string target_host,
			   string multicast_group = "239.255.42.99",
			   int command_port = 1510, int data_port = 1511);
	void update();

	void sendPing();
    void sendRequestDescription();

	bool isConnected();
	int getFrameNumber() { return frame_number; }
	float getLatency() { return latency; }

	float getDataRate();
	float getLastPacketArraivalTime();

	void setScale(float v);
	ofVec3f getScale();

	void setTransform(const ofMatrix4x4& m);
	const ofMatrix4x4& getTransform();

	void setDuplicatedPointRemovalDistance(float v);

	inline const size_t getNumMarkersSet() { return markers_set.size(); }
	inline const vector<Marker>& getMarkersSetAt(size_t index) { return markers_set[index]; }
	
	inline const size_t getNumMarker() { return markers.size(); }
	inline const Marker& getMarker(size_t index) { return markers[index]; }

	inline const size_t getNumFilterdMarker() { return filterd_markers.size(); }
	inline const Marker& getFilterdMarker(size_t index)
	{
		return filterd_markers[index];
	}

    /*
     * Returns the amount of rigidbody descriptions which are received by sendRequestDescription()
    */
    inline const size_t getNumRigidBodyDescriptions() { return rigidbody_descs.size(); }
    /*
     * Returns the amount of rigidbodies received through the data stream
     */
    inline const size_t getNumRigidBody() { return rigidbodies_arr.size(); }

    /*
     * Return the rigidbody at the index in the rigidbody vector
     */
    inline const RigidBody& getRigidBodyAt(int index)
	{
        return rigidbodies_arr[index];
	}

    /*
     * Returns true if a rigidbody with given id is available
     */
	inline const bool hasRigidBody(int id)
	{
		return rigidbodies.find(id) != rigidbodies.end();
	}
    /*
     * Sets the rigidbody RB to the rigidbody with the given id
     * returns false if the rigidbody is not available
     */
	inline const bool getRigidBody(int id, RigidBody& RB)
	{
		if (!hasRigidBody(id)) return false;
		RB = rigidbodies[id];
		return true;
	}
	
	inline const size_t getNumSkeleton() { return skeletons_arr.size(); }
	inline const Skeleton& getSkeletonAt(int index)
	{
		return skeletons_arr[index];
	}
	
	inline const bool hasSkeleton(int id)
	{
		return skeletons.find(id) != skeletons.end();
	}
	inline const bool getSkeleton(int id, Skeleton& S)
	{
		if (!hasSkeleton(id)) return false;
		S = skeletons[id];
		return true;
	}

	void setBufferTime(float sec);
	float getBufferTime();
	
	void setTimeout(float timeout);

	void forceSetNatNetVersion(int major, int minor);

	void debugDraw();
	void debugDrawInformation();
	void debugDrawMarkers();
    
	inline const vector<MarkerSetDescription> getMarkerSetDescriptions() { return markerset_descs; }
	inline const vector<RigidBodyDescription> getRigidBodyDescriptions() { return rigidbody_descs; }
	inline const vector<SkeletonDescription> getSkeletonDescriptions() { return skeleton_descs; }
    
    static map<string, Poco::Net::IPAddress> getNetworkInterfaces();
protected:
	InternalThread* thread;

	int frame_number;
	float latency;
	float timeout;
	
	vector<vector<Marker> > markers_set;
	vector<Marker> filterd_markers;
	vector<Marker> markers;

	map<int, RigidBody> rigidbodies;
    vector<RigidBody> rigidbodies_arr;
	
	map<int, Skeleton> skeletons;
	vector<Skeleton> skeletons_arr;

	vector<RigidBodyDescription> rigidbody_descs;
	vector<SkeletonDescription> skeleton_descs;
	vector<MarkerSetDescription> markerset_descs;
    
	void dispose();

private:
	ofxNatNet(const ofxNatNet&);
	ofxNatNet& operator=(const ofxNatNet&);
};

//

inline bool ofxNatNet::RigidBody::active() const { return isActive(); }
