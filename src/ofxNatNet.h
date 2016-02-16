#pragma once

#include "ofMain.h"

#include <Poco/Net/DatagramSocket.h>

#include <atomic>

namespace ofxNatNet {

	typedef ofVec3f Marker;

	struct LabeledMarker : public Marker {
		int id;
		float size;
		uint16_t params;

		inline bool isOccluded() const { return params & 0x01; }
		inline bool isPointCloudSolbed() const { return params & 0x02; }
		inline bool isModelSolved() const { return params & 0x04; }
	};

	struct MarkerSet {
		std::string name;
		std::vector<Marker> markers;
	};

	struct RigidBody {
		int id;
		ofMatrix4x4 matrix;
		std::vector<Marker> markers;
		std::vector<int> markerID;
		std::vector<float> markerSize;
		float meanMarkerError;
		bool tracking;
	};

	struct Skeleton {
		int id;
		std::vector<RigidBody> joints;
	};

	struct Frame {
		float timestamp;

		int frameNumber;

		std::vector<MarkerSet> markerSets;
		std::vector<Marker> markers;
		std::vector<LabeledMarker> labeledMarkers;
		std::vector<RigidBody> rigidbodies;
		std::vector<Skeleton> skeletons;

		float latency;
		unsigned int timecode;
		unsigned int timecodeSub;
		double natnetTimestamp;
	};

	struct MarkerSetDescription {
		string name;
		std::vector<string> markerNames;
	};

	struct RigidBodyDescription {
		string name;
		int id;
		int parentID;
		ofVec3f offset;
		std::vector<string> markerNames;
	};

	struct SkeletonDescription {
		string name;
		int id;
		std::vector<RigidBodyDescription> joints;
	};

	class Client
	{
	public:

		Client();
		~Client();

		bool connect(const std::string& interface_ip, const std::string& server_ip, int timeout_ms = 1000);
		void disconnect();

		void sendPing(int timeout_ms = 500);
		void sendRequestModelDef(int timeout_ms = 500);

		void update();
		bool isFrameNew();

		bool getFrame(Frame& frame);

		void draw();

		bool isConnected() const { return connected; }
		float getFps() const { return fps; }
		
		const string& getInterfaceIP() const { return interfaceIP; }
		const string& getServerIP() const { return serverIP; }
		
		const std::vector<MarkerSetDescription>& getMarkerSetDescriptions() const { return markersetDescriptions; }
		const std::vector<RigidBodyDescription>& getRigidbodyDescriptions() const { return rigidbodyDescriptions; }
		const std::vector<SkeletonDescription>& getSkeletonDescriptions() const { return skeletonDescriptions; }

	public:

		void setScale(float scale);

	public:

		ofEvent<Frame> onFrameUpdate;
		ofEvent<Frame> onFrameReceive; // WARNING: called by socket thread

	protected:

		std::atomic_bool isRunning;

		std::unique_ptr<std::thread> dataSocketThread;
		void dataSocketThreadLoop();

		std::unique_ptr<std::thread> commandSocketThread;
		void commandSocketLoop();
		Poco::Net::DatagramSocket commandSocket;
		ofThreadChannel<std::string> commandDataChannel;

		std::deque<std::shared_ptr<Frame>> queue;
		std::mutex mutex;
		std::condition_variable condition;

		std::shared_ptr<Frame> latestFrame;

		bool connected;
		bool frameNew;

		int natnet_version[4];
		int server_version[4];
		int major, minor;

		std::string interfaceIP;
		std::string serverIP;

		int command_port;
		int data_port;

		std::string multicast_group;

		std::vector<MarkerSetDescription> markersetDescriptions;
		std::vector<RigidBodyDescription> rigidbodyDescriptions;
		std::vector<SkeletonDescription> skeletonDescriptions;

		float pingTimer;
		float fps;

		float scale;
		ofMatrix4x4 transform;

		void sendCommandMessage(int message_type);
		bool sendCommandMessageBlocking(int message_type, int timeout_ms);

		bool unpackCommandSocket(const uint8_t* data, size_t size);
		bool unpackPing(const uint8_t* data, size_t size);
		bool unpackModelDef(const uint8_t* data, size_t size);

		bool unpackFrame(uint8_t* data, size_t size, Frame& frame);
		uint8_t* unpackMarkerSet(uint8_t* ptr, vector<Marker>& markers);
		uint8_t* unpackRigidBodies(uint8_t* ptr, std::vector<RigidBody>& rigidbodies);
		bool receiveFrame(std::shared_ptr<Frame>& frame, bool& is_last_frame);
	};

}