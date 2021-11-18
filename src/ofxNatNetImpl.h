#pragma once
#include "ofxNatNet.h"

static constexpr int impl_major = 3;
static constexpr int impl_minor = 1;

#define MAX_NAMELENGTH 256

// NATNET message ids
#define NAT_PING 0
#define NAT_PINGRESPONSE 1
#define NAT_REQUEST 2
#define NAT_RESPONSE 3
#define NAT_REQUEST_MODELDEF 4
#define NAT_MODELDEF 5
#define NAT_REQUEST_FRAMEOFDATA 6
#define NAT_FRAMEOFDATA 7
#define NAT_MESSAGESTRING 8
#define NAT_UNRECOGNIZED_REQUEST 100
#define UNDEFINED 999999.9999

#define MAX_PACKETSIZE 100000
// This is needed to enhance portability for struct padding
// a compiler will pad the struct at will to optimize it
// for the compiled architecture
// ref: https://forums.naturalpoint.com/viewtopic.php?f=59&t=13272
#pragma pack(push,1)

// sender
struct sSender
{
	char szName[MAX_NAMELENGTH];  // sending app's name
	uint8_t
		Version[4];  // sending app's version [major.minor.build.revision]
	uint8_t NatNetVersion
		[4];  // sending app's NatNet version [major.minor.build.revision]
};

struct sPacket
{
	uint16_t iMessage;	// message ID (e.g. NAT_FRAMEOFDATA)
	uint16_t nDataBytes;  // Num bytes in payload
	union
	{
		unsigned char cData[MAX_PACKETSIZE];
		char szData[MAX_PACKETSIZE];
		uint64_t lData[MAX_PACKETSIZE / 4];
		float fData[MAX_PACKETSIZE / 4];
		sSender Sender;
	} Data;  // Payload
};

struct Packet
{
	float timestamp;
	struct sPacket packet;
};

struct ofxNatNet::InternalThread : public ofThread
{
	bool connected;
	string target_host;

	int command_port;

	Poco::Net::NetworkInterface iface;
	Poco::Net::MulticastSocket data_socket;
	Poco::Net::DatagramSocket command_socket;

	unsigned char NatNetVersion[4];
	unsigned char ServerVersion[4];

	size_t frame_number;
	float latency;

	float buffer_time;
	queue<Packet> buffer;

	vector<ofxNatNet::Marker> _markers;
	vector<ofxNatNet::Marker> _filterd_markers;

	vector<ofxNatNet::RigidBody> _rigidbodies_arr;
	map<int, ofxNatNet::RigidBody> _rigidbodies;

	vector<vector<ofxNatNet::Marker> > _markers_set;
	vector<ofxNatNet::Skeleton> _skeletons_arr;
	map<int, ofxNatNet::Skeleton> _skeletons;

	vector<RigidBodyDescription> _rigidbody_descs;
	vector<SkeletonDescription> _skeleton_descs;
	vector<MarkerSetDescription> _markerset_descs;

	map<string, int> _name_to_stream_id;


	float last_packet_arrival_time;
	float data_rate;

	ofMatrix4x4 transform;

	float duplicated_point_removal_distance;

	string error_str;

	InternalThread(string iface_name, string target_host,
		string multicast_group, int command_port, int data_port);

	~InternalThread();

	struct remove_dups
	{
		ofVec3f v;
		float dist;

		remove_dups(const ofVec3f& v, float dist)
			: v(v)
			, dist(dist)
		{
		}

		bool operator()(const ofVec3f& t) { return v.match(t, dist); }
	};

	void threadedFunction();

	void sendRequestDescription();

	void sendPing();

	virtual void dataReceive(float target_time);

	void dataPacketReceiverd(sPacket& packet);

	char* unpackMarkerSet(char* ptr, vector<Marker>& ref_markers);

	char* unpackRigidBodies(char* ptr, vector<RigidBody>& ref_rigidbodies);

	void Unpack(char* pData);
};
#pragma pack(pop)
