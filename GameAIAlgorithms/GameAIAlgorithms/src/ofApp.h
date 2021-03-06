#pragma once

#include "ofMain.h"
struct GraphLargeData;
struct Location;
/*

*/

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
		// Helpers
		void DrawGrid();
		void DrawCircleInCell(int x, int y);
		GraphLargeData ParseLargeDataSet();
		void drawBoid(ofVec2f &pos, float &ori, ofColor &clr);
		void addForest(Location L);

		// Executers
		void ExecuteLargeDataSets();
		void ExecuteCampusMap();
		void ExecuteGridExample();

		// Decision making
		void RunDecisionTree();
		void MakeDecision_ChooseTarget();
};
