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
	
	ofxNatNet::listInterfaces();
	
	natnet.setup("en0", "192.168.0.10"); // interface name, server ip
	natnet.setScale(100);
	natnet.setDuplicatedPointRemovalDistance(20);
}

//--------------------------------------------------------------
void testApp::update()
{
	natnet.update();
}

//--------------------------------------------------------------
void testApp::draw()
{
	ofEnableAlphaBlending();
	
	cam.begin();

	ofDrawAxis(100);

	ofFill();

	// draw all markers
	ofSetColor(255, 30);
	for (int i = 0; i < natnet.getNumMarker(); i++)
	{
		ofDrawBox(natnet.getMarker(i), 3);
	}

	ofNoFill();
	
	// draw filterd markers
	ofSetColor(255);
	for (int i = 0; i < natnet.getNumFilterdMarker(); i++)
	{
		ofDrawBox(natnet.getFilterdMarker(i), 10);
	}

	// draw rigidbodies
	for (int i = 0; i < natnet.getNumRigidBody(); i++)
	{
		const ofxNatNet::RigidBody &RB = natnet.getRigidBodyAt(i);

		if (RB.active())
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
	
	cam.end();

	string str;
	str += "frames: " + ofToString(natnet.getFrameNumber()) + "\n";
	str += "data rate: " + ofToString(natnet.getDataRate()) + "\n";
	str += string("connected: ") + (natnet.isConnected() ? "YES" : "NO") + "\n";
	str += "num marker: " + ofToString(natnet.getNumMarker()) + "\n";
	str += "num filterd (non regidbodies) marker: " + ofToString(natnet.getNumFilterdMarker()) + "\n";
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