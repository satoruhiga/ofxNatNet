#include "ofxNatNet.h"

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

	Poco::Net::NetworkInterface interface;
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
    
	float last_packet_arrival_time;
	float data_rate;

	ofMatrix4x4 transform;

	float duplicated_point_removal_distance;

	string error_str;

	InternalThread(string interface_name, string target_host,
				   string multicast_group, int command_port, int data_port)
		: connected(false)
		, target_host(target_host)
		, command_port(command_port)
		, frame_number(0)
		, latency(0)
		, buffer_time(0)
		, last_packet_arrival_time(0)
		, data_rate(0)
		, duplicated_point_removal_distance(0)
	{
		error_str = "";

		try
		{
			{
				Poco::Net::SocketAddress addr(Poco::Net::IPAddress::wildcard(),
											  data_port);
				                try
				{
					interface = Poco::Net::NetworkInterface::forAddress(Poco::Net::IPAddress(interface_name));
				}
				catch (const Poco::Net::InvalidAddressException& e)
				{
					interface = Poco::Net::NetworkInterface::forName(
						interface_name, Poco::Net::NetworkInterface::IPv4_ONLY);
				}
                

				data_socket.bind(addr, true);
				data_socket.joinGroup(Poco::Net::IPAddress(multicast_group),
									  interface);

				data_socket.setBlocking(false);

				data_socket.setReceiveBufferSize(0x100000);
#ifndef TARGET_LINUX    //on linux buffer sizes are restricted so this will fail
                assert(data_socket.getReceiveBufferSize() == 0x100000);
#endif
            }

			for (int i = 0; i < 4; i++)
			{
				NatNetVersion[i] = 0;
				ServerVersion[i] = 0;
			}

			{

                
                
#ifndef TARGET_OS_X      //on OSX interface.address() returns an ipv6 address 
                Poco::Net::SocketAddress my_addr("0.0.0.0", 0);
#else
                Poco::Net::SocketAddress my_addr(interface.address(), 0);
#endif
				command_socket.bind(my_addr, true);
				command_socket.setReceiveBufferSize(0x100000);
				command_socket.setBroadcast(true);
#ifndef TARGET_LINUX    //on linux buffer sizes are restricted so this will fail
                assert(command_socket.getReceiveBufferSize() == 0x100000);
#endif
			}

			{
				Poco::Net::SocketAddress target_addr(target_host, command_port);
				command_socket.connect(target_addr);
				command_socket.setSendBufferSize(0x100000);
#ifndef TARGET_LINUX    //on linux buffer sizes are restricted so this will fail
                assert(command_socket.getSendBufferSize() == 0x100000);
#endif
			}

            sendPing();

            startThread();

		}
		catch (const std::exception& e)
		{
			ofLogError("ofxNatNet") << e.what();
			error_str = e.what();
		}
	}

	~InternalThread()
	{
		if (isThreadRunning()) waitForThread(true);

		data_socket.close();
	}

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

	void threadedFunction()
	{
		Poco::Timespan timeout(0);

		while (isThreadRunning())
		{
			float t = ofGetElapsedTimef();

			if (data_socket.poll(timeout, Poco::Net::Socket::SELECT_READ))
			{
				try
				{
					Packet packet;
					int n = data_socket.receiveBytes((char*)&packet.packet,
													 sizeof(sPacket));

					if (n > 0)
					{
						packet.timestamp = t;
						buffer.push(packet);

						float d = t - last_packet_arrival_time;
						float r = (1. / d);

						data_rate += (r - data_rate) * 0.1;
						last_packet_arrival_time = t;
					}
				}
				catch (Poco::Exception& exc)
				{
					ofLogError("ofxNatNet")
						<< "udp socket error: " << exc.displayText();
				}
			}

			float target_time = t - buffer_time;
			while (buffer.size())
			{
				Packet& packet = buffer.front();
				if (packet.timestamp >= target_time)
				{
					break;
				}

				dataPacketReceiverd(packet.packet);
				buffer.pop();
			}


			ofSleepMillis(1);
		}
	}
    
    void sendRequestDescription() {
        sPacket packet;
        packet.iMessage = NAT_REQUEST_MODELDEF;
        packet.nDataBytes = 0;
        
        Poco::Timespan timeout(100 * 1000);
        
        for (int i = 0; i < 3; i++)
        {
            unsigned int n = command_socket.sendBytes(&packet, 4 + packet.nDataBytes);
            
			if (command_socket.poll(timeout, Poco::Net::Socket::SELECT_READ))
			{
				command_socket.receiveBytes((char*)&packet, sizeof(sPacket));
				if (packet.nDataBytes > 0) {
					Unpack((char*)&packet);
				}
			}
        }
    }

	void sendPing()
	{
		sPacket ping_packet;
		ping_packet.iMessage = NAT_PING;
		ping_packet.nDataBytes = 0;

		connected = false;
		
		Poco::Timespan timeout(100 * 1000);

		for (int i = 0; i < 3; i++)
		{
			unsigned int n = command_socket.sendBytes(
				&ping_packet, 4 + ping_packet.nDataBytes);
			if (n > 0 && connected) break;

			if (command_socket.poll(timeout, Poco::Net::Socket::SELECT_READ))
			{
				try
				{
					sPacket packet;
					int n = command_socket.receiveBytes((char*)&packet,
														sizeof(sPacket));
					

					if (n > 0 && packet.iMessage == NAT_PINGRESPONSE)
					{
						connected = true;

                        for (int i = 0; i < 4; i++)
						{
                            NatNetVersion[i] = packet.Data.Sender.NatNetVersion[i];
                            ServerVersion[i] = packet.Data.Sender.Version[i];
                        }

						printf("connected. NatNet: v%i.%i, Server: v%i.%i\n", NatNetVersion[0], NatNetVersion[1], ServerVersion[0], ServerVersion[1]);
                        fflush(stdout);

						return;
					}
				}
				catch (Poco::Exception& exc)
				{
					ofLogError("ofxNatNet")
					<< "udp socket error: " << exc.displayText();
				}
			}

			ofLogWarning("ofxNatNet") << "No route to host. count: " << i;
		}
	}
	
	void dataPacketReceiverd(sPacket& packet)
	{
		Unpack((char*)&packet);
	}
	
    char* unpackMarkerSet(char* ptr, vector<Marker>& ref_markers)
	{
		int nMarkers = 0;
		memcpy(&nMarkers, ptr, 4);
		ptr += 4;
		
        ref_markers.resize(nMarkers);
		
		for (int j = 0; j < nMarkers; j++)
		{
			ofVec3f p;
			memcpy(&p.x, ptr, 4);
			ptr += 4;
			memcpy(&p.y, ptr, 4);
			ptr += 4;
			memcpy(&p.z, ptr, 4);
			ptr += 4;

			p = transform.preMult(p);
			
            ref_markers[j] = p;
		}
		
		return ptr;
	}

    char* unpackRigidBodies(char* ptr, vector<RigidBody>& ref_rigidbodies)
	{
        int major = NatNetVersion[0];
        int minor = NatNetVersion[1];
        //it was --> int major = NatNetVersion[0];
        //it was --> int minor = NatNetVersion[1];
		
		ofQuaternion rot = transform.getRotate();

		int nRigidBodies = 0;
		memcpy(&nRigidBodies, ptr, 4);
		ptr += 4;

        ref_rigidbodies.resize(nRigidBodies);
		
		for (int j = 0; j < nRigidBodies; j++)
		{
            ofxNatNet::RigidBody& RB = ref_rigidbodies[j];
			
			ofVec3f pp;
			ofQuaternion q;
			
			int ID = 0;
			memcpy(&ID, ptr, 4);
			ptr += 4;
			
			memcpy(&pp.x, ptr, 4);
			ptr += 4;
			
			memcpy(&pp.y, ptr, 4);
			ptr += 4;
			
			memcpy(&pp.z, ptr, 4);
			ptr += 4;
			
			memcpy(&q.x(), ptr, 4);
			ptr += 4;
			
			memcpy(&q.y(), ptr, 4);
			ptr += 4;
			
			memcpy(&q.z(), ptr, 4);
			ptr += 4;
			
			memcpy(&q.w(), ptr, 4);
			ptr += 4;
			
			RB.id = ID;
			RB.raw_position = pp;
			
			pp = transform.preMult(pp);
			
			ofMatrix4x4 mat;
			mat.setTranslation(pp);
			mat.setRotate(q * rot);
			RB.matrix = mat;
			
			// associated marker positions
			int nRigidMarkers = 0;
			memcpy(&nRigidMarkers, ptr, 4);
			ptr += 4;
			
			int nBytes = nRigidMarkers * 3 * sizeof(float);
			float* markerData = (float*)malloc(nBytes);
			memcpy(markerData, ptr, nBytes);
			ptr += nBytes;
			
			if (major >= 2)
			{
				// associated marker IDs
				nBytes = nRigidMarkers * sizeof(int);
				ptr += nBytes;
				
				// associated marker sizes
				nBytes = nRigidMarkers * sizeof(float);
				ptr += nBytes;
			}
			
			RB.markers.resize(nRigidMarkers);
			
			for (int k = 0; k < nRigidMarkers; k++)
			{
				float x = markerData[k * 3];
				float y = markerData[k * 3 + 1];
				float z = markerData[k * 3 + 2];
				
				ofVec3f pp(x, y, z);
				pp = transform.preMult(pp);
				RB.markers[k] = pp;
			}
			
			if (markerData) free(markerData);
			
			if (major >= 2)
			{
				// Mean marker error
				float fError = 0.0f;
				memcpy(&fError, ptr, 4);
				ptr += 4;
				
				RB.mean_marker_error = fError;
				RB._active = RB.mean_marker_error > 0;
			} else {
				RB.mean_marker_error = 0;
			}

			// 2.6 and later
			if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0))
			{
				// params
				short params = 0; memcpy(&params, ptr, 2); ptr += 2;
				bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
			}
			
		}  // next rigid body
		
		return ptr;
	}
	
	void Unpack(char* pData)
	{
        int major = NatNetVersion[0];
        int minor = NatNetVersion[1];
        //it was --> int major = NatNetVersion[0];
        //it was --> int minor = NatNetVersion[1];
		
        if (major == 0 && minor == 0)
		{
			ofLogError("ofxNatNet") << "initialize failed";
			return;
        }
		
		if (major > impl_major || minor > impl_minor)
		{
			ofLogError("ofxNatNet") << "The implemented NatNet parser is outdated";
			return;
		}

		ofQuaternion rot = transform.getRotate();

		char* ptr = pData;

		// message ID
		int MessageID = 0;
		memcpy(&MessageID, ptr, 2);
		ptr += 2;

		// size
		int nBytes = 0;
		memcpy(&nBytes, ptr, 2);
		ptr += 2;

		if (MessageID == 7)  // FRAME OF MOCAP DATA packet
		{
			int frame_number = 0;
			float latency = 0;

            vector<vector<Marker> > tmp_markers_set;
            vector<Skeleton> tmp_skeletons;
            vector<Marker> tmp_markers;
            vector<Marker> tmp_filterd_markers;
            vector<RigidBody> tmp_rigidbodies;

			// frame number
			memcpy(&frame_number, ptr, 4);
			ptr += 4;

			// number of data sets (markersets, rigidbodies, etc)
			int nMarkerSets = 0;
			memcpy(&nMarkerSets, ptr, 4);
			ptr += 4;
			
            tmp_markers_set.resize(nMarkerSets);
			
			for (int i = 0; i < nMarkerSets; i++)
            {
				// Markerset name
				char szName[256];
				strcpy(szName, ptr);
				int nDataBytes = (int)strlen(szName) + 1;
				ptr += nDataBytes;
				
                ptr = unpackMarkerSet(ptr, tmp_markers_set[i]);
			}

			// unidentified markers
            ptr = unpackMarkerSet(ptr, tmp_markers);

			// rigid bodies
            ptr = unpackRigidBodies(ptr, tmp_rigidbodies);

			if (((major == 2) && (minor > 0)) || (major > 2)) {
				int nSkeletons = 0;
				memcpy(&nSkeletons, ptr, 4);
				ptr += 4;
				
                tmp_skeletons.resize(nSkeletons);

				for (int j = 0; j < nSkeletons; j++) {
					int skeletonID = 0;
					memcpy(&skeletonID, ptr, 4);
					ptr += 4;
					
                    tmp_skeletons[j].id = skeletonID;
					
                    ptr = unpackRigidBodies(ptr, tmp_skeletons[j].joints);
				}
			}

			// labeled markers (version 2.3 and later)
			if (((major == 2) && (minor >= 3)) || (major > 2)) {
				int nLabeledMarkers = 0;
				memcpy(&nLabeledMarkers, ptr, 4);
				ptr += 4;
				for (int j = 0; j < nLabeledMarkers; j++) {
					// id
					int ID = 0;
					memcpy(&ID, ptr, 4);
					ptr += 4;
					// x
					float x = 0.0f;
					memcpy(&x, ptr, 4);
					ptr += 4;
					// y
					float y = 0.0f;
					memcpy(&y, ptr, 4);
					ptr += 4;
					// z
					float z = 0.0f;
					memcpy(&z, ptr, 4);
					ptr += 4;
					// size
					float size = 0.0f;
					memcpy(&size, ptr, 4);
					ptr += 4;

					// 2.6 and later
					if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) )
					{
						// marker params
						short params = 0; memcpy(&params, ptr, 2); ptr += 2;
						bool bOccluded = params & 0x01;     // marker was not visible (occluded) in this frame
						bool bPCSolved = params & 0x02;     // position provided by point cloud solve
						bool bModelSolved = params & 0x04;  // position provided by model solve
					}

					ofVec3f pp(x, y, z);
					pp = transform.preMult(pp);

                    tmp_markers.push_back(pp);
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
			memcpy(&latency, ptr, 4);
			ptr += 4;

			this->latency = latency;

			// timecode
			unsigned int timecode = 0; 	memcpy(&timecode, ptr, 4);	ptr += 4;
			unsigned int timecodeSub = 0; memcpy(&timecodeSub, ptr, 4); ptr += 4;

			// timestamp
			double timestamp = 0.0f;
			// 2.7 and later - increased from single to double precision
			if (((major == 2) && (minor >= 7)) || (major>2))
			{
				memcpy(&timestamp, ptr, 8); ptr += 8;
			}
			else
			{
				float fTemp = 0.0f;
				memcpy(&fTemp, ptr, 4); ptr += 4;
			}

			// frame params
			short params = 0;  memcpy(&params, ptr, 2); ptr += 2;
			bool bIsRecording = params & 0x01;                  // 0x01 Motive is recording
			bool bTrackedModelsChanged = params & 0x02;         // 0x02 Actively tracked model list has changed

			// end of data tag
			int eod = 0;
			memcpy(&eod, ptr, 4);
			ptr += 4;

            tmp_filterd_markers = tmp_markers;

			// filter markers
			if (duplicated_point_removal_distance > 0)
			{
				map<int, ofxNatNet::RigidBody>::iterator it =
                    this->_rigidbodies.begin();
                while (it != this->_rigidbodies.end())
				{
					ofxNatNet::RigidBody& RB = it->second;

					for (int i = 0; i < RB.markers.size(); i++)
					{
						ofVec3f& v = RB.markers[i];
						vector<Marker>::iterator it = remove_if(
                            tmp_filterd_markers.begin(), tmp_filterd_markers.end(),
							remove_dups(v, duplicated_point_removal_distance));
                        tmp_filterd_markers.erase(it, tmp_filterd_markers.end());
					}

					it++;
				}
			}

			// copy to mainthread
			if (lock())
			{
				this->latency = latency;
				this->frame_number = frame_number;
                this->_markers_set = tmp_markers_set;
                this->_markers = tmp_markers;
                this->_filterd_markers = tmp_filterd_markers;
                this->_rigidbodies_arr = tmp_rigidbodies;
                this->_skeletons_arr = tmp_skeletons;
                // fill the rigidbodies map
                {
                    for (int i = 0; i < tmp_rigidbodies.size(); i++) {
                        RigidBody &RB = tmp_rigidbodies[i];
                        RigidBody &tRB = this->_rigidbodies[RB.id];
						tRB = RB;
					}
				}
				{
                    for (int i = 0; i < tmp_skeletons.size(); i++) {
                        Skeleton &S = tmp_skeletons[i];
                        Skeleton &tS = this->_skeletons[S.id];
						tS = S;
					}
				}

				unlock();
			}
		}
		else if (MessageID == 5)  // Data Descriptions
		{
            int nDatasets = 0;

            vector<RigidBodyDescription> tmp_rigidbody_descs;
            vector<SkeletonDescription> tmp_skeleton_descs;
            vector<MarkerSetDescription> tmp_markerset_descs;

			// number of datasets
			memcpy(&nDatasets, ptr, 4);
			ptr += 4;

			for (int i = 0; i < nDatasets; i++)
			{
				int type = 0;
				memcpy(&type, ptr, 4);
				ptr += 4;

				if (type == 0)   // markerset
				{
                    MarkerSetDescription description;
                    
					// name
					char szName[256];
					strcpy(szName, ptr);
					int nDataBytes = (int)strlen(szName) + 1;
					ptr += nDataBytes;
                    description.name = szName;

					// marker data
					int nMarkers = 0;
					memcpy(&nMarkers, ptr, 4);
					ptr += 4;

					for (int j = 0; j < nMarkers; j++)
					{
						char szName[256];
						strcpy(szName, ptr);
						int nDataBytes = (int)strlen(szName) + 1;
						ptr += nDataBytes;
                        description.marker_names.push_back(szName);
					}
                    tmp_markerset_descs.push_back(description);
				}
				else if (type == 1)   // rigid body
				{
                    RigidBodyDescription description;
                    
					if (major >= 2)
					{
						// name
						char szName[MAX_NAMELENGTH];
						strcpy(szName, ptr);
						ptr += strlen(ptr) + 1;
                        description.name = szName;
					}

					int ID = 0;
					memcpy(&ID, ptr, 4);
					ptr += 4;
                    description.id = ID;

					int parentID = 0;
					memcpy(&parentID, ptr, 4);
					ptr += 4;
                    description.parent_id = parentID;

					float xoffset = 0;
					memcpy(&xoffset, ptr, 4);
					ptr += 4;

					float yoffset = 0;
					memcpy(&yoffset, ptr, 4);
					ptr += 4;

					float zoffset = 0;
					memcpy(&zoffset, ptr, 4);
					ptr += 4;

                    description.offset.x = xoffset;
                    description.offset.y = yoffset;
                    description.offset.z = zoffset;
                    
                    tmp_rigidbody_descs.push_back(description);
				}
				else if (type == 2)   // skeleton
				{
                    SkeletonDescription description;
                    
					char szName[MAX_NAMELENGTH];
					strcpy(szName, ptr);
					ptr += strlen(ptr) + 1;
                    description.name = szName;

					int ID = 0;
					memcpy(&ID, ptr, 4);
					ptr += 4;
                    description.id = ID;

					int nRigidBodies = 0;
					memcpy(&nRigidBodies, ptr, 4);
					ptr += 4;
                    
                    description.joints.resize(nRigidBodies);

					for (int i = 0; i < nRigidBodies; i++)
					{
						if (major >= 2)
						{
							// RB name
							char szName[MAX_NAMELENGTH];
							strcpy(szName, ptr);
							ptr += strlen(ptr) + 1;
                            description.joints[i].name = szName;
						}

						int ID = 0;
						memcpy(&ID, ptr, 4);
						ptr += 4;
                        description.joints[i].id = ID;

						int parentID = 0;
						memcpy(&parentID, ptr, 4);
						ptr += 4;
                        description.joints[i].parent_id = parentID;

						float xoffset = 0;
						memcpy(&xoffset, ptr, 4);
						ptr += 4;

						float yoffset = 0;
						memcpy(&yoffset, ptr, 4);
						ptr += 4;

						float zoffset = 0;
						memcpy(&zoffset, ptr, 4);
						ptr += 4;
                        
                        description.joints[i].offset.x = xoffset;
                        description.joints[i].offset.y = yoffset;
                        description.joints[i].offset.z = zoffset;
					}
                    tmp_skeleton_descs.push_back(description);
				}

			}   // next dataset
            
            // copy to mainthread
            if (lock())
            {
                this->_markerset_descs = tmp_markerset_descs;
                this->_rigidbody_descs = tmp_rigidbody_descs;
                this->_skeleton_descs = tmp_skeleton_descs;
                unlock();
            }
		}
		else
		{
			ofLogError("ofxNatNet") << "Unrecognized Packet Type";
		}
	}
};

void ofxNatNet::setup(string interface_name, string target_host,
					  string multicast_group, int command_port, int data_port)
{
	dispose();
	thread = new InternalThread(interface_name, target_host, multicast_group,
								command_port, data_port);
}

void ofxNatNet::dispose()
{
	if (thread) delete thread;
	thread = NULL;
}

void ofxNatNet::update()
{
	if (thread == NULL)
	{
		ofLogError("ofxNatNet") << "call setup() first";
		return;
	}

	if (thread->lock())
	{
		frame_number = thread->frame_number;
		latency = thread->latency;
		
		if (isConnected())
		{
            markers_set = thread->_markers_set;
            markers = thread->_markers;
            filterd_markers = thread->_filterd_markers;
			
			{
                rigidbodies = thread->_rigidbodies;
				rigidbodies_arr.clear();
                rigidbodies_arr = thread->_rigidbodies_arr;

			}
			{
                skeletons = thread->_skeletons;
				skeletons_arr.clear();
                skeletons_arr = thread->_skeletons_arr;
                
			}
            
            markerset_descs = thread->_markerset_descs;
            rigidbody_descs = thread->_rigidbody_descs;
            skeleton_descs = thread->_skeleton_descs;
		}
		else
		{
			markers_set.clear();
			markers.clear();
			filterd_markers.clear();
			rigidbodies.clear();
			rigidbodies_arr.clear();
			skeletons.clear();
			skeletons_arr.clear();
		}

		thread->unlock();
	}
}

bool ofxNatNet::isConnected()
{
	if (!thread) return false;
	return thread->connected
		&& (ofGetElapsedTimef() - thread->last_packet_arrival_time) < this->timeout;
}

float ofxNatNet::getDataRate()
{
	if (!thread) return 0;
	return thread->data_rate;
}

float ofxNatNet::getLastPacketArraivalTime()
{
	if (!thread) return 0;
	return thread->last_packet_arrival_time;
}

void ofxNatNet::setScale(float v)
{
	assert(thread);
	thread->transform = ofMatrix4x4::newScaleMatrix(v, v, v);
}

ofVec3f ofxNatNet::getScale()
{
	assert(thread);
	return thread->transform.getScale();
}

void ofxNatNet::setDuplicatedPointRemovalDistance(float v)
{
	assert(thread);
	if (v < 0) v = 0;
	thread->duplicated_point_removal_distance = v;
}

void ofxNatNet::setBufferTime(float sec)
{
	assert(thread);
	thread->buffer_time = ofClamp(sec, 0, 10);
}

float ofxNatNet::getBufferTime()
{
	assert(thread);
	return thread->buffer_time;
}

void ofxNatNet::setTimeout(float timeout)
{
	this->timeout = timeout;
}

void ofxNatNet::forceSetNatNetVersion(int major, int minor)
{
	assert(thread);
	thread->NatNetVersion[0] = major;
	thread->NatNetVersion[1] = minor;
}

void ofxNatNet::sendPing() { thread->sendPing(); }

void ofxNatNet::sendRequestDescription() { thread->sendRequestDescription(); }

void ofxNatNet::setTransform(const ofMatrix4x4& m)
{
	assert(thread);
	thread->transform = m;
}

const ofMatrix4x4& ofxNatNet::getTransform()
{
	assert(thread);
	return thread->transform;
}

void ofxNatNet::debugDrawMarkers()
{
	ofPushStyle();

	ofFill();

	// draw all markers
	ofSetColor(255, 30);
	for (int i = 0; i < getNumMarker(); i++)
	{
		ofDrawBox(getMarker(i), 3);
	}

	ofNoFill();

	// draw filterd markers
	ofSetColor(255);
	for (int i = 0; i < getNumFilterdMarker(); i++)
	{
		ofDrawBox(getFilterdMarker(i), 10);
	}

	// draw rigidbodies
	for (int i = 0; i < getNumRigidBody(); i++)
	{
		const ofxNatNet::RigidBody& RB = getRigidBodyAt(i);

		if (RB.isActive())
			ofSetColor(0, 255, 0);
		else
			ofSetColor(255, 0, 0);

		ofPushMatrix();
		glMultMatrixf(RB.getMatrix().getPtr());
		ofDrawAxis(30);
		ofPopMatrix();

		glBegin(GL_LINE_LOOP);
		for (int n = 0; n < RB.markers.size(); n++)
		{
			glVertex3fv(RB.markers[n].getPtr());
		}
		glEnd();

		for (int n = 0; n < RB.markers.size(); n++)
		{
			ofDrawBox(RB.markers[n], 5);
		}
	}
	
	// draw skeletons
	for (int j = 0;  j < getNumSkeleton(); j++) {
		const ofxNatNet::Skeleton &S = getSkeletonAt(j);
		ofSetColor(255, 0, 255);
		
		for (int i = 0; i < S.joints.size(); i++) {
			const ofxNatNet::RigidBody &RB = S.joints[i];
			ofPushMatrix();
			glMultMatrixf(RB.getMatrix().getPtr());
			ofDrawBox(5);
			ofPopMatrix();
		}
	}

	ofPopStyle();
}

void ofxNatNet::debugDrawInformation()
{
	ofPushStyle();
	
	string str;
	if (thread->error_str != "") str += "ERROR: " + thread->error_str + "\n";
	str += "frames: " + ofToString(getFrameNumber()) + "\n";
	str += "data rate: " + ofToString(getDataRate()) + "\n";
	str += string("connected: ") + (isConnected() ? "YES" : "NO") + "\n";
	str += "num marker: " + ofToString(getNumMarker()) + "\n";
	str += "num filterd (non rigidbodies) marker: " +
	ofToString(getNumFilterdMarker()) + "\n";
	str += "num rigidbody: " + ofToString(getNumRigidBody()) + "\n";
	str += "num skeleton: " + ofToString(getNumSkeleton()) + "\n\n";
    
    if (markerset_descs.size() || rigidbody_descs.size() || skeleton_descs.size()) {
        str += "Description: \n";
        for (auto& desc : markerset_descs) { str += "MarkerSet Name: " + desc.name + "\n"; }
        for (auto& desc : rigidbody_descs) { str += "RigidBody Name: " + desc.name + "\n"; }
        for (auto& desc : skeleton_descs) { str += "Skeleton Name: " + desc.name + "\n"; }
    }
	
	ofDrawBitmapStringHighlight(str, 10, 20, ofColor(40), ofColor(255));
	
	ofPopStyle();
}

void ofxNatNet::debugDraw()
{
	debugDrawMarkers();
	debugDrawInformation();
}

map<string, Poco::Net::IPAddress> ofxNatNet::getNetworkInterfaces()
{
    map<string, Poco::Net::IPAddress> ret;

    Poco::Net::NetworkInterface::Map m = Poco::Net::NetworkInterface::map(true, true);
    assert (!m.empty());
    for (Poco::Net::NetworkInterface::Map::const_iterator it = m.begin(); it != m.end(); ++it)
    {
        std::cout << std::endl << "=============" << std::endl;

        std::cout << "Index:       " << it->second.index() << std::endl;
        std::cout << "Name:        " << it->second.name() << std::endl;
        std::cout << "DisplayName: " << it->second.displayName() << std::endl;
        std::cout << "Status:      " << (it->second.isUp() ? "Up" : "Down") << std::endl;

        Poco::Net::NetworkInterface::MACAddress mac(it->second.macAddress());
        if (!mac.empty() && (it->second.type() != Poco::Net::NetworkInterface::NI_TYPE_SOFTWARE_LOOPBACK))
                std::cout << "MAC Address: (" << it->second.type() << ") " << mac << std::endl;

        typedef Poco::Net::NetworkInterface::AddressList List;

        const List& ipList = it->second.addressList();
        List::const_iterator ipIt = ipList.begin();
        List::const_iterator ipEnd = ipList.end();
        for (int counter = 0; ipIt != ipEnd; ++ipIt, ++counter)
        {
            int fam = ipIt->get<Poco::Net::NetworkInterface::IP_ADDRESS>().family();

            std::cout << std::endl << "----------" << std::endl;
            std::cout << "Address " << counter << std::endl;
            std::cout << "----------" << std::endl;
            std::cout << "Family:     " << fam << std::endl;
            std::cout << "Address:     " << ipIt->get<Poco::Net::NetworkInterface::IP_ADDRESS>() << std::endl;
            Poco::Net::IPAddress addr = ipIt->get<Poco::Net::NetworkInterface::SUBNET_MASK>();
            if (!addr.isWildcard()) std::cout << "Subnet:      " << addr << " (/" << addr.prefixLength() << ")" << ")" << std::endl;
                    addr = ipIt->get<Poco::Net::NetworkInterface::BROADCAST_ADDRESS>();
            if (!addr.isWildcard()) std::cout << "Broadcast:   " << addr << std::endl;
            if (fam == Poco::Net::AddressFamily::IPv4)
            {
                ret[it->second.name()] = ipIt->get<Poco::Net::NetworkInterface::IP_ADDRESS>();
            }
        }
    }
    return ret;
}
