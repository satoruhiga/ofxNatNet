#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofBackground(0);
	ofSetLogLevel(OF_LOG_VERBOSE);
	natnet.connect("127.0.0.1", "127.0.0.1");
}

//--------------------------------------------------------------
void ofApp::update(){
	natnet.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
	cam.begin();
	ofDrawAxis(100);

	natnet.draw();

	cam.end();

	stringstream ss;
	ss << "### NatNet Status" << endl;
	ss << "* connected: " << (natnet.isConnected() ? "YES" : "NO") << endl;

	if (natnet.isConnected())
	{
		ss << "* interface ip: " << natnet.getInterfaceIP() << endl;
		ss << "* server ip: " << natnet.getServerIP() << endl;
		ss << "* receive fps: " << natnet.getFps();

		ss << endl << endl;

		ofxNatNet::Frame frame;
		if (natnet.getFrame(frame))
		{
			ss << "### Frame Status" << endl;
			ss << "* timestamp: " << frame.timestamp << endl;

			ss << "* num unlabeled markers: " << frame.markers.size() << endl;
			ss << "* num labeled markers: " << frame.labeledMarkers.size() << endl;
			ss << "* num marker sets: " << frame.markerSets.size() << endl;
			ss << "* num rigidbodies: " << frame.rigidbodies.size() << endl;
			for (const auto& RB : frame.rigidbodies)
			{
				ss << "\tid: " << RB.id << endl;
				ss << "\tname: " << RB.name << endl;
				ss << "\t:tracking: " << (RB.tracking ? "YES" : "NO") << endl;
				ss << "\t:mean error: " << RB.meanMarkerError << endl;
			}

			ss << "* timecode: " << frame.timecode << endl;
			ss << "* timecodeSub: " << frame.timecodeSub << endl;
			ss << "* natnetTimestamp: " << frame.natnetTimestamp;
		}
	}

	ofDrawBitmapStringHighlight(ss.str(), 10, 20);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == '1')
	{
		natnet.connect("127.0.0.1", "127.0.0.1");
	}

	if (key == '0')
	{
		natnet.disconnect();
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
