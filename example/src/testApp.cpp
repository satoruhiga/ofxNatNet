#include "testApp.h"

#include "ofxNatNet.h"

ofxNatNet natnet;
ofEasyCam cam;

//--------------------------------------------------------------
void testApp::setup()
{
	ofSetFrameRate(60);
	ofSetVerticalSync(true);
	ofBackground(0);

	natnet.setup(); // setup with default network device
	// natnet.setup("192.168.1.10"); // setup with network device ip
	natnet.setScale(100);
}

//--------------------------------------------------------------
void testApp::update()
{
	natnet.update();
}

//--------------------------------------------------------------
void testApp::draw()
{
	cam.begin();

	ofDrawAxis(100);

	ofNoFill();

	ofSetColor(255, 0, 0);
	for (int i = 0; i < natnet.getNumMarker(); i++)
	{
		ofBox(natnet.getMarker(i), 10);
	}

	for (int i = 0; i < natnet.getNumRigidBody(); i++)
	{
		const ofxNatNet::RigidBody &RB = natnet.getRigidBody(i);

		ofPushMatrix();
		glMultMatrixf(RB.getMatrix().getPtr());
		ofDrawAxis(30);
		ofPopMatrix();

		ofSetColor(0, 255, 0);

		for (int n = 0; n < RB.markers.size(); n++)
		{
			ofBox(RB.markers[n], 5);
		}
	}


	cam.end();

	string str;
	str += "frames: " + ofToString(natnet.getFrameNumber()) + "\n";
	str += "data rate: " + ofToString(natnet.getDataRate()) + "\n";
	str += string("connected: ") + (natnet.isConnected() ? "YES" : "NO") + "\n";
	str += "num marker: " + ofToString(natnet.getNumMarker()) + "\n";
	str += "num markerset: " + ofToString(natnet.getNumMarkerSet()) + "\n";
	str += "num rigidbody: " + ofToString(natnet.getNumRigidBody()) + "\n";

	ofSetColor(255);
	ofDrawBitmapString(str, 10, 20);
}

//--------------------------------------------------------------
void testApp::keyPressed(int key)
{

}

//--------------------------------------------------------------
void testApp::keyReleased(int key)
{

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y)
{

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg)
{

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo)
{

}