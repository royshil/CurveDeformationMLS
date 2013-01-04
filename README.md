CurveDeformationMLS
===================

2D Shape/Curve deformation using Moving Least Sqaures

http://www.morethantechnical.com/2013/01/05/shape-manipulation-with-moving-least-squares-for-curves-w-code/

Build
-----
	mkdir build
	cd build
	cmake ..
	make

API
---
	//Read curve
	vector<Point> a;
	GetCurveForImage(imread("a_slihouette.png", a, false);

	//Convert to Point_<double> - optional
	vector<Point2d> a_p2d, a_p2d_smoothed;
	ConvertCurve(a, a_p2d);

	//Get curvature extrema points
	vector<pair<char,int> > stringrep = CurvatureExtrema(a_p2d, a_p2d_smoothed,0.05,4.0);

	//Get extrema as control points
	vector<int> control_pts; 
	for(int i=0;i<stringrep.size();i++) {
		control_pts.push_back(stringrep[i].second);
	}

	smls.Init(a_p2d, control_pts);
	smls.UpdateRigid();
		
	Mat visualized_curve(500,500,CV_8UC3);
	smls.Draw(visualized_curve);

	namedWindow("MLS");
	//Implement the onMouse function (or take from repo) to reflect changes in control points
	setMouseCallback("MLS", onMouse, NULL);
	imshow("MLS", visualized_curve);
	waitKey();

