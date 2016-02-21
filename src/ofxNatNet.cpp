#include "ofxNatNet.h"

#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/MulticastSocket.h>
#include <Poco/Net/NetworkInterface.h>
#include <Poco/Net/NetException.h>

const int impl_major = 2;
const int impl_minor = 9;

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

struct sSender {
	char szName[MAX_NAMELENGTH];  // sending app's name
	unsigned char
		Version[4];  // sending app's version [major.minor.build.revision]
	unsigned char NatNetVersion
		[4];  // sending app's NatNet version [major.minor.build.revision]
};

struct sPacket
{
	unsigned short iMessage;	// message ID (e.g. NAT_FRAMEOFDATA)
	unsigned short nDataBytes;  // Num bytes in payload
	union {
		unsigned char cData[MAX_PACKETSIZE];
		char szData[MAX_PACKETSIZE];
		unsigned long lData[MAX_PACKETSIZE / 4];
		float fData[MAX_PACKETSIZE / 4];
		sSender Sender;
	} Data;  // Payload
};

namespace ofxNatNet {

	Client::Client()
		: isRunning(false)
		, connected(false)
		, frameNew(false)
		, data_port(1511)
		, command_port(1510)
		, multicast_group("239.255.42.99")
		, fps(0)
		, pingTimer(-1)
	{
		setTransform(ofMatrix4x4::newScaleMatrix(ofVec3f(100)));
	}

	Client::~Client()
	{
		disconnect();
	}

	bool Client::connect(const std::string& interface_ip, const std::string& server_ip, int timeout_ms)
	{
		disconnect();

		this->interfaceIP = interface_ip;
		this->serverIP = server_ip;

		isRunning = true;

		commandSocketThread = std::make_unique<std::thread>([this]() {
			this->commandSocketLoop();
		});

		dataSocketThread = std::make_unique<std::thread>([this]() {
			this->dataSocketThreadLoop();
		});

		//

		sendPing(timeout_ms);

		if (!connected)
		{
			ofLogError("ofxNatNet") << "connection refused";
			return false;
		}

		ofLogVerbose("ofxNatNet") << "connected NatNet: " << natnet_version[0] << "." << natnet_version[1]
			<< " Server: " << server_version[0] << "." << server_version[1];

		return true;
	}

	void Client::disconnect()
	{
		if (isRunning == false) return;

		isRunning = false;

		{
			std::unique_lock<std::mutex> lock(mutex);
			queue.clear();
			condition.notify_all();
		}

		dataSocketThread->join();
		dataSocketThread.reset();

		commandSocketThread->join();
		commandSocketThread.reset();

		commandSocket.close();

		connected = false;

		for (int i = 0; i < 4; i++)
		{
			natnet_version[i] = 0;
			server_version[i] = 0;
		}

		interfaceIP.clear();
		serverIP.clear();

		fps = 0;
	}

	void Client::sendPing(int timeout_ms)
	{
		sendCommandMessageBlocking(NAT_PING, timeout_ms);
	}

	void Client::sendRequestModelDef(int timeout_ms)
	{
		sendCommandMessageBlocking(NAT_REQUEST_MODELDEF, timeout_ms);
	}

	void Client::sendCommandMessage(int message_type)
	{
		sPacket request_packet;
		request_packet.iMessage = message_type;
		request_packet.nDataBytes = 0;

		Poco::Net::SocketAddress server_addr(Poco::Net::IPAddress(serverIP), command_port);

		commandSocket.sendTo(&request_packet, 4 + request_packet.nDataBytes, server_addr);
	}

	bool Client::sendCommandMessageBlocking(int message_type, int timeout_ms)
	{
		Poco::Net::DatagramSocket socket;

		socket.bind(Poco::Net::SocketAddress(interfaceIP, 0), true);
		socket.setSendBufferSize(0x100000);
		socket.setReceiveBufferSize(0x100000);
		socket.setBroadcast(true);

		sPacket request_packet;
		request_packet.iMessage = message_type;
		request_packet.nDataBytes = 0;

		for (int i = 0; i < 3; i++)
		{
			Poco::Net::SocketAddress server_addr(Poco::Net::IPAddress(serverIP), command_port);

			socket.sendTo(&request_packet, 4 + request_packet.nDataBytes, server_addr);

			try
			{
				if (socket.poll(Poco::Timespan(timeout_ms * 1000), Poco::Net::Socket::SELECT_READ))
				{
					sPacket packet;
					int n = socket.receiveBytes(&packet, sizeof(sPacket));
					if (unpackCommandSocket((const uint8_t*)&packet, n))
						return true;
				}

			}
			catch (const Poco::Net::NetException& e)
			{
				ofLogError("ofxNatNet") << i << ": " << e.what();
			}

			ofSleepMillis(1000);
		}

		return false;
	}

	void Client::commandSocketLoop()
	{
		int timeout_ms = 100;

		commandSocket.bind(Poco::Net::SocketAddress(interfaceIP, 0), true);
		commandSocket.setSendBufferSize(0x100000);
		commandSocket.setReceiveBufferSize(0x100000);
		commandSocket.setBroadcast(true);

		while (isRunning)
		{
			try
			{
				if (commandSocket.poll(Poco::Timespan(timeout_ms * 1000), Poco::Net::Socket::SELECT_READ))
				{
					sPacket packet;
					int n = commandSocket.receiveBytes(&packet, sizeof(sPacket));

					if (n > 0)
					{
						commandDataChannel.send(std::string((char*)&packet, n));
					}
				}
			}
			catch (const Poco::Net::NetException& e)
			{
				ofLogError("ofxNatNet") << e.what();
			}
		}
	}

	bool Client::unpackCommandSocket(const uint8_t* data, size_t size)
	{
		sPacket* packet = (sPacket*)data;
		
		switch (packet->iMessage)
		{
			case NAT_PINGRESPONSE:
				return unpackPing(data, size);
				break;
			case NAT_MODELDEF:
				return unpackModelDef(data, size);
				break;
			default:
				break;
		}

		return false;
	}

	bool Client::unpackPing(const uint8_t* data, size_t size)
	{
		sPacket* packet = (sPacket*)data;

		for (int i = 0; i < 4; i++)
		{
			this->natnet_version[i] = packet->Data.Sender.NatNetVersion[i];
			this->server_version[i] = packet->Data.Sender.Version[i];
		}

		major = natnet_version[0];
		minor = natnet_version[1];

		this->connected = true;

		return true;
	}

	bool Client::unpackModelDef(const uint8_t* data, size_t size)
	{
		sPacket* packet = (sPacket*)data;
		const uint8_t* ptr = (const uint8_t*)&packet->Data;

		std::lock_guard<std::mutex> lock(modeDefMutex);

		markersetDescriptions.clear();
		rigidbodyDescriptions.clear();
		skeletonDescriptions.clear();

		// number of datasets
		int nDatasets = 0; memcpy(&nDatasets, ptr, 4); ptr += 4;

		for (int i = 0; i < nDatasets; i++)
		{
			int type = 0; memcpy(&type, ptr, 4); ptr += 4;

			switch (type)
			{
				case 0: // marker set
				{
					markersetDescriptions.emplace_back();
					auto& desc = markersetDescriptions.back();

					desc.name = (char*)ptr;
					ptr += desc.name.size() + 1;

					// marker data
					int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
					desc.markerNames.resize(nMarkers);
					for (int n = 0; n < nMarkers; n++)
					{
						desc.markerNames[n] = (char*)ptr;
						ptr += desc.markerNames[n].size() + 1;
					}

					break;
				}
				case 1: // rigid body
				{
					RigidBodyDescription desc;

					if (major >= 2)
					{
						desc.name = (char*)ptr;
						ptr += desc.name.size() + 1;
					}

					memcpy(&desc.id, ptr, 4); ptr += 4;
					memcpy(&desc.parentID, ptr, 4); ptr += 4;
					memcpy(&desc.offset, ptr, sizeof(ofVec3f)); ptr += sizeof(ofVec3f);

					rigidbodyDescriptions[desc.id] = desc;

					break;
				}
				case 2: // skeleton
				{
					SkeletonDescription desc;

					desc.name = (char*)ptr;
					ptr += desc.name.size() + 1;

					memcpy(&desc.id, ptr, 4); ptr += 4;

					int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr += 4;
					desc.joints.resize(nRigidBodies);

					for (int n = 0; n < nRigidBodies; n++)
					{
						auto& RBD = desc.joints[n];

						if (major >= 2)
						{
							desc.name = (char*)ptr;
							ptr += RBD.name.size() + 1;
						}

						memcpy(&RBD.id, ptr, 4); ptr += 4;
						memcpy(&RBD.parentID, ptr, 4); ptr += 4;
						memcpy(&RBD.offset, ptr, sizeof(ofVec3f)); ptr += sizeof(ofVec3f);
					}

					skeletonDescriptions[desc.id] = desc;

					break;
				}
			}
		}

		if (std::distance((const uint8_t*)&packet->Data, (const uint8_t*)ptr) != packet->nDataBytes)
		{
			ofLogNotice("ofxNatNet") << "invalid packet size. might be parse error?";
			return false;
		}

		return true;
	}

	void Client::dataSocketThreadLoop()
	{
		Poco::Net::MulticastSocket socket(Poco::Net::SocketAddress(Poco::Net::IPAddress::wildcard(), data_port), true);
		Poco::Net::NetworkInterface interface = Poco::Net::NetworkInterface::forAddress(Poco::Net::IPAddress(interfaceIP));
		socket.joinGroup(Poco::Net::IPAddress(multicast_group), interface);

		socket.setReceiveBufferSize(0x100000);

		int timeout_ms = 100;
		float last_packet_arrival_time = 0;

		while (isRunning)
		{
			if (socket.poll(Poco::Timespan(timeout_ms * 1000), Poco::Net::Socket::SELECT_READ))
			{
				float T = ofGetElapsedTimef();

				try
				{
					sPacket packet;
					size_t n = socket.receiveBytes(&packet, sizeof(sPacket));

					if (n > 0 && packet.iMessage == NAT_FRAMEOFDATA)
					{
						std::shared_ptr<Frame> frame = std::make_shared<Frame>();

						if (unpackFrame((uint8_t*)&packet, n, *frame.get()))
						{
							frame->timestamp = T;

							{
								std::unique_lock<std::mutex> lock(mutex);
								if (isRunning)
								{
									queue.push_back(frame);
									if (queue.size() > 100)
										queue.pop_front();
									condition.notify_all();
								}
							}

							float d = T - last_packet_arrival_time;
							float r = (1. / d);
							fps += (r - fps) * 0.1;

							last_packet_arrival_time = T;

							ofNotifyEvent(onFrameReceive, *frame.get());
						}
					}
				}
				catch (const Poco::Net::NetException& e)
				{
					ofLogError("ofxNatNet") << e.what();
				}
			}
			else
			{
				float T = ofGetElapsedTimef();
				float d = T - last_packet_arrival_time;
				float r = (1. / d);
				fps += (r - fps) * 0.1;
			}
		}
	}

	bool Client::unpackFrame(uint8_t* data, size_t size, Frame& frame)
	{
		if (major == 0 && minor == 0)
		{
			ofLogError("ofxNatNet") << "initialize failed";
			return false;
		}

		if (major > impl_major || minor > impl_minor)
		{
			ofLogError("ofxNatNet") << "The implemented NatNet parser is outdated";
			return false;
		}
		
		sPacket* packet = (sPacket*)data;
		if (packet->iMessage != NAT_FRAMEOFDATA)
			return false;

		uint8_t* ptr = (uint8_t*)&packet->Data;

		memcpy(&frame.frameNumber, ptr, 4);
		ptr += 4;

		// number of data sets (marker sets, rigidbodies, etc)
		int nMarkerSets = 0;
		memcpy(&nMarkerSets, ptr, 4);
		ptr += 4;

		for (int i = 0; i < nMarkerSets; i++)
		{
			std::string name;
			name = (char*)ptr;
			ptr += name.size() + 1;

			MarkerSet& o = frame.markerSets[name];
			o.name = name;
			ptr = unpackMarkerSet(ptr, o.markers);
		}

		// unidentified markers
		ptr = unpackMarkerSet(ptr, frame.markers);

		// rigid bodies
		int nRigidBodies = 0;
		memcpy(&nRigidBodies, ptr, 4);

		ptr = unpackRigidBodies(ptr, frame.rigidbodies);

		if (((major == 2) && (minor > 0)) || (major > 2)) {
			int nSkeletons = 0;
			memcpy(&nSkeletons, ptr, 4);
			ptr += 4;

			frame.skeletons.resize(nSkeletons);

			for (int i = 0; i < nSkeletons; i++) {
				Skeleton& o = frame.skeletons[i];

				memcpy(&o.id, ptr, 4);
				ptr += 4;

				ptr = unpackRigidBodies(ptr, o.joints);

				std::lock_guard<std::mutex> lock(modeDefMutex);

				const auto& it = skeletonDescriptions.find(o.id);
				if (it != skeletonDescriptions.end())
				{
					o.name = it->second.name;
				}
				else
				{
					o.name = "(UNKNOWN)";
				}
			}
		}

		// labeled markers (version 2.3 and later)
		if (((major == 2) && (minor >= 3)) || (major > 2)) {
			int nLabeledMarkers = 0;
			memcpy(&nLabeledMarkers, ptr, 4);
			ptr += 4;

			frame.labeledMarkers.resize(nLabeledMarkers);

			for (int i = 0; i < nLabeledMarkers; i++) {
				LabeledMarker& o = frame.labeledMarkers[i];
				Marker& p = o;

				// id
				memcpy(&o.id, ptr, 4);
				ptr += 4;

				memcpy(p.getPtr(), ptr, sizeof(ofVec3f));
				p = transform.preMult(p);
				ptr += sizeof(ofVec3f);

				// size
				memcpy(&o.size, ptr, 4);
				ptr += 4;

				// 2.6 and later
				if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0))
				{
					// marker params
					o.params = 0;
					memcpy(&o.params, ptr, 2);
					ptr += 2;
				}
			}
		}

		// Force Plate data (version 2.9 and later)
		if (((major == 2) && (minor >= 9)) || (major > 2))
		{
			int nForcePlates;
			memcpy(&nForcePlates, ptr, 4); ptr += 4;
			for (int iForcePlate = 0; iForcePlate < nForcePlates; iForcePlate++)
			{
				// ID
				int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;

				// Channel Count
				int nChannels = 0; memcpy(&nChannels, ptr, 4); ptr += 4;

				// Channel Data
				for (int i = 0; i < nChannels; i++)
				{
					int nFrames = 0; memcpy(&nFrames, ptr, 4); ptr += 4;
					for (int j = 0; j < nFrames; j++)
					{
						float val = 0.0f;  memcpy(&val, ptr, 4); ptr += 4;
					}
				}
			}
		}

		// latency
		memcpy(&frame.latency, ptr, 4);
		ptr += 4;

		// timecode
		memcpy(&frame.timecode, ptr, 4); ptr += 4;
		memcpy(&frame.timecodeSub, ptr, 4); ptr += 4;

		// timestamp
		
		// 2.7 and later - increased from single to double precision
		if (((major == 2) && (minor >= 7)) || (major>2))
		{
			double timestamp = 0.0f;
			memcpy(&timestamp, ptr, 8); ptr += 8;
			frame.natnetTimestamp = timestamp;
		}
		else
		{
			float timestamp = 0.0f;
			memcpy(&timestamp, ptr, 4); ptr += 4;
			frame.natnetTimestamp = timestamp;
		}

		// frame params
		short params = 0;  memcpy(&params, ptr, 2); ptr += 2;
		bool bIsRecording = params & 0x01; // 0x01 Motive is recording
		bool bTrackedModelsChanged = params & 0x02; // 0x02 Actively tracked model list has changed

		// end of data tag
		int eod = 0;
		memcpy(&eod, ptr, 4);
		ptr += 4;

		if (std::distance((uint8_t*)&packet->Data, ptr) != packet->nDataBytes)
		{
			ofLogNotice("ofxNatNet") << "invalid packet size. might be parse error?";
			return false;
		}

		return true;
	}

	uint8_t* Client::unpackMarkerSet(uint8_t* ptr, vector<Marker>& markers)
	{
		int nMarkers = 0;
		memcpy(&nMarkers, ptr, 4);
		ptr += 4;

		markers.resize(nMarkers);

		for (int i = 0; i < nMarkers; i++)
		{
			memcpy(markers[i].getPtr(), ptr, sizeof(ofVec3f));
			markers[i] = transform.preMult(markers[i]);
			ptr += sizeof(ofVec3f);
		}

		return ptr;
	}

	uint8_t* Client::unpackRigidBodies(uint8_t* ptr, std::vector<RigidBody>& rigidbodies)
	{
		int nRigidBodies = 0;
		memcpy(&nRigidBodies, ptr, 4);
		ptr += 4;

		rigidbodies.resize(nRigidBodies);

		for (int i = 0; i < nRigidBodies; i++)
		{
			RigidBody& o = rigidbodies[i];

			memcpy(&o.id, ptr, 4);
			ptr += 4;

			ofVec3f p;
			ofQuaternion q;

			memcpy(p.getPtr(), ptr, sizeof(ofVec3f));
			ptr += sizeof(ofVec3f);

			memcpy(q._v.getPtr(), ptr, sizeof(ofVec4f));
			ptr += sizeof(ofVec4f);

			p = transform.preMult(p);

			o.matrix.setTranslation(p);
			o.matrix.setRotate(q);

			int nRigidMarkers = 0;
			memcpy(&nRigidMarkers, ptr, 4);
			ptr = unpackMarkerSet(ptr, o.markers);

			if (major >= 2)
			{
				int nBytes;

				// associated marker IDs
				nBytes = nRigidMarkers * sizeof(int);
				o.markerID.resize(nRigidMarkers);
				memcpy(o.markerID.data(), ptr, nBytes);
				ptr += nBytes;

				// associated marker sizes
				nBytes = nRigidMarkers * sizeof(float);
				o.markerSize.resize(nRigidMarkers);
				memcpy(o.markerSize.data(), ptr, nBytes);
				ptr += nBytes;

				// Mean marker error
				memcpy(&o.meanMarkerError, ptr, 4);
				ptr += 4;
			}
			else {
				o.meanMarkerError = 0;
			}

			// 2.6 and later
			if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0))
			{
				// params
				short params = 0; memcpy(&params, ptr, 2); ptr += 2;
				o.tracking = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
			}
			else
			{
				o.tracking = o.meanMarkerError > 0;
			}

			std::lock_guard<std::mutex> lock(modeDefMutex);

			const auto& it = rigidbodyDescriptions.find(o.id);
			if (it != rigidbodyDescriptions.end())
			{
				o.name = it->second.name;
			}
			else
			{
				o.name = "(UNKNOWN)";
			}
		}

		return ptr;
	}

	bool Client::getFrame(Frame& frame)
	{
		if (latestFrame)
		{
			frame = *latestFrame.get();
			return true;
		}
		else return false;
	}

	void Client::update(float frame_timeout)
	{
		pingTimer -= ofGetLastFrameTime();
		if (pingTimer < 0)
		{
			pingTimer = 10;

			sendCommandMessage(NAT_PING);
			sendCommandMessage(NAT_REQUEST_MODELDEF);
		}

		{
			std::string data;
			while (commandDataChannel.tryReceive(data))
			{
				unpackCommandSocket((const uint8_t*)data.data(), data.size());
			}
		}
		
		frameNew = false;

		std::shared_ptr<Frame> frame;

		{
			std::unique_lock<std::mutex> lock(mutex);

			while (!queue.empty())
			{
				frame = queue.front();
				queue.pop_front();

				frameNew = true;
			}
		}

		if (frame)
		{
			latestFrame = frame;
			ofNotifyEvent(onFrameUpdate, *latestFrame.get());
		}

		if (latestFrame && (ofGetElapsedTimef() - latestFrame->timestamp) > frame_timeout)
		{
			latestFrame.reset();
		}
	}

	bool Client::isFrameNew()
	{
		return frameNew;
	}

	void Client::draw()
	{
		if (!latestFrame) return;

		Frame frame = *latestFrame.get();

		ofPushStyle();
		ofNoFill();

		ofSetColor(0, 255, 0);
		for (const auto& p : frame.markers)
		{
			ofDrawBox(p, 2);
		}

		ofSetColor(0, 0, 255);
		for (const auto& p : frame.labeledMarkers)
		{
			ofDrawBox(p, 4);
		}

		for (const auto& RB : frame.rigidbodies)
		{
			ofPushMatrix();

			ofSetColor(0, 255, 255);
			for (const auto& p : RB.markers)
			{
				ofDrawBox(p, 6);
			}

			ofSetColor(RB.tracking ? ofColor(255) : ofColor(255, 0, 0));

			glBegin(GL_LINE_LOOP);
			for (const auto& p : RB.markers)
			{
				glVertex3fv(p.getPtr());
			}
			glEnd();

			ofSetColor(255);

			ofPushMatrix();
			ofMultMatrix(RB.matrix);
			ofDrawBitmapString(std::to_string(RB.id) + ":" + RB.name, ofVec3f(0));
			ofDrawAxis(30);
			ofPopMatrix();

			ofPopMatrix();
		}

		ofPopStyle();
	}

	void Client::setTransform(const ofMatrix4x4& transform)
	{
		this->transform = transform;
	}

}