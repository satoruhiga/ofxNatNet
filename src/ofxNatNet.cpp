#include "ofxNatNet.h"

#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/MulticastSocket.h>
#include <Poco/Net/NetworkInterface.h>

#define MAX_NAMELENGTH              256

// NATNET message ids
#define NAT_PING                    0
#define NAT_PINGRESPONSE            1
#define NAT_REQUEST                 2
#define NAT_RESPONSE                3
#define NAT_REQUEST_MODELDEF        4
#define NAT_MODELDEF                5
#define NAT_REQUEST_FRAMEOFDATA     6
#define NAT_FRAMEOFDATA             7
#define NAT_MESSAGESTRING           8
#define NAT_UNRECOGNIZED_REQUEST    100
#define UNDEFINED                   999999.9999

// sender
typedef struct
{
	char szName[MAX_NAMELENGTH]; // sending app's name
	unsigned char Version[4]; // sending app's version [major.minor.build.revision]
	unsigned char NatNetVersion[4]; // sending app's NatNet version [major.minor.build.revision]

} sSender;

typedef struct
{
	unsigned short iMessage; // message ID (e.g. NAT_FRAMEOFDATA)
	unsigned short nDataBytes; // Num bytes in payload
	union
	{
		unsigned char cData[20000];
		char szData[20000];
		unsigned long lData[5000];
		float fData[5000];
		sSender Sender;
	} Data; // Payload

} sPacket;

struct ofxNatNet::InternalThread : public ofThread
{
	bool connected;
	string target_host;

	int command_port;

	Poco::Net::DatagramSocket command_socket;
	Poco::Net::MulticastSocket data_socket;

	int NatNetVersion[4];
	int ServerVersion[4];

	size_t frame_number;
	float latency;
	
	int buffer_size;
	queue<sPacket> buffer;

	vector<ofxNatNet::Marker> markers;
	vector<ofxNatNet::MarkerSet> markersets;
	vector<ofxNatNet::RigidBody> rigidbodies;
	
	float last_packet_received;
	float data_rate;
	
	float scale;
	float duplicated_point_removal_distance;

	InternalThread(string target_host, string multicast_group, int command_port, int data_port) : connected(false), target_host(target_host), command_port(command_port), frame_number(0), latency(0), scale(1), buffer_size(0), last_packet_received(0), data_rate(0), duplicated_point_removal_distance(0)
	{
		{
			Poco::Net::IPAddress ip(multicast_group);

			Poco::Net::SocketAddress addr(Poco::Net::IPAddress(ip.family()), data_port);
			data_socket.bind(addr, true);
			data_socket.joinGroup(ip);

			data_socket.setBlocking(false);
			
			data_socket.setReceiveBufferSize(0x100000);
			assert(data_socket.getReceiveBufferSize() == 0x100000);
		}

		{
			command_socket.bind(Poco::Net::SocketAddress(Poco::Net::IPAddress::wildcard(), command_port));
			
			command_socket.setBlocking(false);
			
			command_socket.setBroadcast(true);
			assert(command_socket.getBroadcast());

			command_socket.setSendBufferSize(0x100000);
			assert(command_socket.getSendBufferSize() == 0x100000);
		}
		
		for (int i = 0; i < 4; i++)
		{
			NatNetVersion[i] = 0;
			ServerVersion[i] = 0;
		}
		
		startThread();

		sendPing(target_host);
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
		
		remove_dups(const ofVec3f& v, float dist) : v(v), dist(dist) {}
		
		bool operator()(const ofVec3f &t)
		{
			return v.match(t, dist);
		}
	};

	void threadedFunction()
	{
		Poco::Timespan timeout(0);

		while (isThreadRunning())
		{
			if (data_socket.poll(timeout, Poco::Net::Socket::SELECT_READ))
			{
				try
				{
					sPacket packet;
					int n = data_socket.receiveBytes((char*)&packet, sizeof(sPacket));

					if (n > 0)
					{
						buffer.push(packet);
						
						float t = ofGetElapsedTimef();
						float d = t - last_packet_received;
						float r = (1. / d);
						
						data_rate += (r - data_rate) * 0.1;
						last_packet_received = t;
					}
				}
				catch (Poco::Exception& exc)
				{
					ofLogError("ofxNatNet") << "udp socket error: " << exc.displayText();
				}
			}
			
			while (buffer.size() > buffer_size)
			{
				sPacket &packet = buffer.front();
				dataPacketReceiverd(packet);
				buffer.pop();
			}

			if (command_socket.poll(timeout, Poco::Net::Socket::SELECT_READ))
			{
				try
				{
					sPacket packet;
					int n = command_socket.receiveBytes((char*)&packet, sizeof(sPacket));

					if (n > 0)
						commandPacketReceived(packet);
				}
				catch (Poco::Exception& exc)
				{
					ofLogError("ofxNatNet") << "udp socket error: " << exc.displayText();
				}
			}
			
			ofSleepMillis(1);
		}
	}

	void sendPing(string host)
	{
		sPacket ping_packet;
		ping_packet.iMessage = NAT_PING;
		ping_packet.nDataBytes = 0;

		Poco::Net::SocketAddress addr(Poco::Net::IPAddress(host), command_port);

		connected = false;

		for (int i = 0; i < 10; i++)
		{
			unsigned int n = command_socket.sendTo(&ping_packet, 4 + ping_packet.nDataBytes, addr);
			if (n > 0) break;

			sleep(100);

			ofLogWarning("ofxNatNet") << "No route to host count: " << i;
		}
	}

	void dataPacketReceiverd(sPacket &packet)
	{
		Unpack((char*)&packet);
	}

	void commandPacketReceived(sPacket &packet)
	{
		if (packet.iMessage == NAT_PINGRESPONSE)
		{
			connected = true;

			for (int i = 0; i < 4; i++)
			{
				NatNetVersion[i] = (int)packet.Data.Sender.NatNetVersion[i];
				ServerVersion[i] = (int)packet.Data.Sender.Version[i];
			}
		}
	}

	void Unpack(char* pData)
	{
		int major = NatNetVersion[0];
		int minor = NatNetVersion[1];
		
		if (major == 0 && minor == 0)
		{
			return;
		}
		
		char *ptr = pData;

		// message ID
		int MessageID = 0;
		memcpy(&MessageID, ptr, 2);
		ptr += 2;

		// size
		int nBytes = 0;
		memcpy(&nBytes, ptr, 2);
		ptr += 2;

		if (MessageID == 7)      // FRAME OF MOCAP DATA packet
		{
			int frame_number = 0;
			float latency = 0;

			vector<Marker> markers;
			vector<MarkerSet> markersets;
			vector<RigidBody> rigidbodies;

			// frame number
			memcpy(&frame_number, ptr, 4);
			ptr += 4;

			// number of data sets (markersets, rigidbodies, etc)
			int nMarkerSets = 0;
			memcpy(&nMarkerSets, ptr, 4);
			ptr += 4;

			markersets.resize(nMarkerSets);

			for (int i = 0; i < nMarkerSets; i++)
			{
				ofxNatNet::MarkerSet &MS = markersets[i];

				// Markerset name
				char szName[256];
				strcpy(szName, ptr);
				int nDataBytes = (int)strlen(szName) + 1;
				ptr += nDataBytes;
				MS.name = szName;

				// marker data
				int nMarkers = 0;
				memcpy(&nMarkers, ptr, 4);
				ptr += 4;

				MS.markers.resize(nMarkers);

				for (int j = 0; j < nMarkers; j++)
				{
					float x = 0;
					memcpy(&x, ptr, 4);
					ptr += 4;
					float y = 0;
					memcpy(&y, ptr, 4);
					ptr += 4;
					float z = 0;
					memcpy(&z, ptr, 4);
					ptr += 4;

					MS.markers[j].set(x * scale, y * scale, z * scale);
				}
			}

			// unidentified markers
			int nOtherMarkers = 0;
			memcpy(&nOtherMarkers, ptr, 4);
			ptr += 4;

			markers.resize(nOtherMarkers);

			for (int j = 0; j < nOtherMarkers; j++)
			{
				float x = 0.0f;
				memcpy(&x, ptr, 4);
				ptr += 4;
				float y = 0.0f;
				memcpy(&y, ptr, 4);
				ptr += 4;
				float z = 0.0f;
				memcpy(&z, ptr, 4);
				ptr += 4;

				markers[j].set(x * scale, y * scale, z * scale);
			}

			// rigid bodies
			int nRigidBodies = 0;
			memcpy(&nRigidBodies, ptr, 4);
			ptr += 4;

			rigidbodies.resize(nRigidBodies);

			for (int j = 0; j < nRigidBodies; j++)
			{
				ofxNatNet::RigidBody &RB = rigidbodies[j];

				// rigid body pos/ori
				int ID = 0;
				memcpy(&ID, ptr, 4);
				ptr += 4;
				float x = 0.0f;
				memcpy(&x, ptr, 4);
				ptr += 4;
				float y = 0.0f;
				memcpy(&y, ptr, 4);
				ptr += 4;
				float z = 0.0f;
				memcpy(&z, ptr, 4);
				ptr += 4;
				float qx = 0;
				memcpy(&qx, ptr, 4);
				ptr += 4;
				float qy = 0;
				memcpy(&qy, ptr, 4);
				ptr += 4;
				float qz = 0;
				memcpy(&qz, ptr, 4);
				ptr += 4;
				float qw = 0;
				memcpy(&qw, ptr, 4);
				ptr += 4;
				
				RB.id = ID;
				
				ofMatrix4x4 mat;
				mat.setTranslation(x * scale, y * scale, z * scale);
				mat.setRotate(ofQuaternion(qx, qy, qz, qw));
				RB.matrix = mat;
				
				// associated marker positions
				int nRigidMarkers = 0;
				memcpy(&nRigidMarkers, ptr, 4);
				ptr += 4;

				RB.markers.resize(nRigidMarkers);

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
				
				for (int k = 0; k < nRigidMarkers; k++)
				{
					ofVec3f v(markerData[k * 3] * scale,
							  markerData[k * 3 + 1] * scale,
							  markerData[k * 3 + 2] * scale);
					
					if (duplicated_point_removal_distance > 0)
					{
						vector<Marker>::iterator it = remove_if(markers.begin(), markers.end(), remove_dups(v, duplicated_point_removal_distance));
						markers.erase(it, markers.end());
					}
					
					RB.markers[k].set(v);
				}

				if (markerData)
					free(markerData);

				if (major >= 2)
				{
					// Mean marker error
					float fError = 0.0f;
					memcpy(&fError, ptr, 4);
					ptr += 4;

					RB.mean_marker_error = fError;
				}

			} // next rigid body

			// TODO: parse skeleton

			/*
			// skeletons
			if (((major == 2) && (minor > 0)) || (major > 2))
			{
				int nSkeletons = 0;
				memcpy(&nSkeletons, ptr, 4);
				ptr += 4;
				printf("Skeleton Count : %d\n", nSkeletons);
				for (int j = 0; j < nSkeletons; j++)
				{
					// skeleton id
					int skeletonID = 0;
					memcpy(&skeletonID, ptr, 4);
					ptr += 4;
					// # of rigid bodies (bones) in skeleton
					int nRigidBodies = 0;
					memcpy(&nRigidBodies, ptr, 4);
					ptr += 4;
					printf("Rigid Body Count : %d\n", nRigidBodies);
					for (int j = 0; j < nRigidBodies; j++)
					{
						// rigid body pos/ori
						int ID = 0;
						memcpy(&ID, ptr, 4);
						ptr += 4;
						float x = 0.0f;
						memcpy(&x, ptr, 4);
						ptr += 4;
						float y = 0.0f;
						memcpy(&y, ptr, 4);
						ptr += 4;
						float z = 0.0f;
						memcpy(&z, ptr, 4);
						ptr += 4;
						float qx = 0;
						memcpy(&qx, ptr, 4);
						ptr += 4;
						float qy = 0;
						memcpy(&qy, ptr, 4);
						ptr += 4;
						float qz = 0;
						memcpy(&qz, ptr, 4);
						ptr += 4;
						float qw = 0;
						memcpy(&qw, ptr, 4);
						ptr += 4;
						printf("ID : %d\n", ID);
						printf("pos: [%3.2f,%3.2f,%3.2f]\n", x, y, z);
						printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx, qy, qz, qw);

						// associated marker positions
						int nRigidMarkers = 0;
						memcpy(&nRigidMarkers, ptr, 4);
						ptr += 4;
						printf("Marker Count: %d\n", nRigidMarkers);
						int nBytes = nRigidMarkers * 3 * sizeof(float);
						float* markerData = (float*)malloc(nBytes);
						memcpy(markerData, ptr, nBytes);
						ptr += nBytes;

						// associated marker IDs
						nBytes = nRigidMarkers * sizeof(int);
						int* markerIDs = (int*)malloc(nBytes);
						memcpy(markerIDs, ptr, nBytes);
						ptr += nBytes;

						// associated marker sizes
						nBytes = nRigidMarkers * sizeof(float);
						float* markerSizes = (float*)malloc(nBytes);
						memcpy(markerSizes, ptr, nBytes);
						ptr += nBytes;

						for (int k = 0; k < nRigidMarkers; k++)
						{
							printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k], markerData[k * 3], markerData[k * 3 + 1], markerData[k * 3 + 2]);
						}

						// Mean marker error
						float fError = 0.0f;
						memcpy(&fError, ptr, 4);
						ptr += 4;
						printf("Mean marker error: %3.2f\n", fError);

						// release resources
						if (markerIDs)
							free(markerIDs);
						if (markerSizes)
							free(markerSizes);
						if (markerData)
							free(markerData);

					} // next rigid body

				} // next skeleton
			}
			*/

			// latency
			memcpy(&latency, ptr, 4);
			ptr += 4;

			this->latency = latency;

			// end of data tag
			int eod = 0;
			memcpy(&eod, ptr, 4);
			ptr += 4;

			// copy to mainthread

			if (lock())
			{
				this->latency = latency;
				this->frame_number = frame_number;
				this->markers = markers;
				this->markersets = markersets;
				this->rigidbodies = rigidbodies;

				unlock();
			}
		}
		else if (MessageID == 5) // Data Descriptions
		{
			// TODO: impl description

			/*

			// number of datasets
			int nDatasets = 0;
			memcpy(&nDatasets, ptr, 4);
			ptr += 4;
			printf("Dataset Count : %d\n", nDatasets);

			for (int i = 0; i < nDatasets; i++)
			{
				printf("Dataset %d\n", i);

				int type = 0;
				memcpy(&type, ptr, 4);
				ptr += 4;
				printf("Type : %d\n", i, type);

				if (type == 0)   // markerset
				{
					// name
					char szName[256];
					strcpy(szName, ptr);
					int nDataBytes = (int)strlen(szName) + 1;
					ptr += nDataBytes;
					printf("Markerset Name: %s\n", szName);

					// marker data
					int nMarkers = 0;
					memcpy(&nMarkers, ptr, 4);
					ptr += 4;
					printf("Marker Count : %d\n", nMarkers);

					for (int j = 0; j < nMarkers; j++)
					{
						char szName[256];
						strcpy(szName, ptr);
						int nDataBytes = (int)strlen(szName) + 1;
						ptr += nDataBytes;
						printf("Marker Name: %s\n", szName);
					}
				}
				else if (type == 1)   // rigid body
				{
					if (major >= 2)
					{
						// name
						char szName[MAX_NAMELENGTH];
						strcpy(szName, ptr);
						ptr += strlen(ptr) + 1;
						printf("Name: %s\n", szName);
					}

					int ID = 0;
					memcpy(&ID, ptr, 4);
					ptr += 4;
					printf("ID : %d\n", ID);

					int parentID = 0;
					memcpy(&parentID, ptr, 4);
					ptr += 4;
					printf("Parent ID : %d\n", parentID);

					float xoffset = 0;
					memcpy(&xoffset, ptr, 4);
					ptr += 4;
					printf("X Offset : %3.2f\n", xoffset);

					float yoffset = 0;
					memcpy(&yoffset, ptr, 4);
					ptr += 4;
					printf("Y Offset : %3.2f\n", yoffset);

					float zoffset = 0;
					memcpy(&zoffset, ptr, 4);
					ptr += 4;
					printf("Z Offset : %3.2f\n", zoffset);

				}
				else if (type == 2)   // skeleton
				{
					char szName[MAX_NAMELENGTH];
					strcpy(szName, ptr);
					ptr += strlen(ptr) + 1;
					printf("Name: %s\n", szName);

					int ID = 0;
					memcpy(&ID, ptr, 4);
					ptr += 4;
					printf("ID : %d\n", ID);

					int nRigidBodies = 0;
					memcpy(&nRigidBodies, ptr, 4);
					ptr += 4;
					printf("RigidBody (Bone) Count : %d\n", nRigidBodies);

					for (int i = 0; i < nRigidBodies; i++)
					{
						if (major >= 2)
						{
							// RB name
							char szName[MAX_NAMELENGTH];
							strcpy(szName, ptr);
							ptr += strlen(ptr) + 1;
							printf("Rigid Body Name: %s\n", szName);
						}

						int ID = 0;
						memcpy(&ID, ptr, 4);
						ptr += 4;
						printf("RigidBody ID : %d\n", ID);

						int parentID = 0;
						memcpy(&parentID, ptr, 4);
						ptr += 4;
						printf("Parent ID : %d\n", parentID);

						float xoffset = 0;
						memcpy(&xoffset, ptr, 4);
						ptr += 4;
						printf("X Offset : %3.2f\n", xoffset);

						float yoffset = 0;
						memcpy(&yoffset, ptr, 4);
						ptr += 4;
						printf("Y Offset : %3.2f\n", yoffset);

						float zoffset = 0;
						memcpy(&zoffset, ptr, 4);
						ptr += 4;
						printf("Z Offset : %3.2f\n", zoffset);
					}
				}

			}   // next dataset

			 */
		}
		else
		{
			ofLogError("ofxNatNet") << "Unrecognized Packet Type";
		}

	}
};

void ofxNatNet::setup(string target_host, string multicast_group, int command_port, int data_port)
{
	dispose();
	thread = new InternalThread(target_host, multicast_group, command_port, data_port);
}

void ofxNatNet::dispose()
{
	if (thread)
		delete thread;
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
		markers = thread->markers;
		markersets = thread->markersets;
		rigidbodies = thread->rigidbodies;
		
		thread->unlock();
	}
}

bool ofxNatNet::isConnected()
{
	if (!thread) return false;
	return thread->connected;
}

float ofxNatNet::getDataRate()
{
	if (!thread) return 0;
	return thread->data_rate;
}

void ofxNatNet::setScale(float v)
{
	assert(thread);
	thread->scale = v;
}

float ofxNatNet::getScale()
{
	assert(thread);
	return thread->scale;
}

void ofxNatNet::setDuplicatedPointRemovalDistance(float v)
{
	assert(thread);
	if (v < 0) v = 0;
	thread->duplicated_point_removal_distance = v;
}

void ofxNatNet::setBufferSize(int n)
{
	assert(thread);
	thread->buffer_size = ofClamp(n, 0, 1000);
}

int ofxNatNet::getBufferSize()
{
	assert(thread);
	return thread->buffer_size;
}
