/*
 *  SchaeferMLS.h
 *  CurveMatching
 *
 *  Created by Roy Shilkrot on 12/28/12.
 *  Copyright 2012 MIT. All rights reserved.
 *
 *	Implementing "Image deformation using moving least squares", S. Schaefer et al. 2006
 *  http://dl.acm.org/citation.cfm?id=1141920
 *	(for 2D curves)
 *
 */

#include <numeric>
#include <functional>

#include "CurveCSS.h"

#define P2V(p) Vec2d((p).x,(p).y)
#define V2P(v) Point2d((v)[0],(v)[1])

template<typename T>
class SchaeferMLS {
	Mat_<double> A;
	vector<vector<Matx22d> > As;
	vector<double> mu_s;
	vector<double> mu_r;
	vector<Point_<T> > control_pts;
	vector<Point_<T> > deformed_control_pts;
	vector<Point_<T> > m_curve;
	vector<Point_<T> > m_original_curve;
	double m_original_curve_scale;
	vector<vector<double> > m_w;
	vector<Vec2d> m_vmp;
	
	template<typename V>
	vector<Point2d> GetCurveAroundMean(const vector<Point_<V> >& p, const Point_<V>& p_star) {
		vector<Point2d> p_hat(p.size());
		for (int i=0; i<p.size(); i++) p_hat[i] = p[i] - p_star;
		return p_hat;
	}
	
	void CalcWeights() {
		m_w.resize(m_curve.size());
		for (int v=0; v<m_curve.size(); v++) {
			m_w[v].resize(control_pts.size());
			for (int i=0; i<control_pts.size(); i++) {
				double d = pow(norm(control_pts[i] - m_curve[v]),3.0);
				if (d < 1.0) {
					m_w[v][i] = 1.0;
				} else {
					m_w[v][i] = 1.0 / d;
				}
			}
		}
	}
	void CalcGeodesicWeights(const vector<int>& ctrl_pts_curve_idx) {
		m_w.resize(m_curve.size());
		for (int v=0; v<m_curve.size(); v++) {
			m_w[v].resize(control_pts.size());
			for (int i=0; i<ctrl_pts_curve_idx.size(); i++) {
				double d = pow(abs(ctrl_pts_curve_idx[i] - v),2.0);
				if (d < 1.0) {
					m_w[v][i] = 1.0;
				} else {
					m_w[v][i] = 1.0 / d;
				}
			}
		}
	}
	
	template<typename Q>
	Point2d GetWeightedMeanForPoint(int v, const vector<Point_<Q> >& q) {
		Point2d q_star(0,0);
		double sum_w_i = 0.0;
		for (int i=0; i<q.size(); i++) {
			q_star += q[i] * m_w[v][i];
			sum_w_i += m_w[v][i];
		}
		
		return q_star * (1.0/sum_w_i);
	}
	
	double GetWeightedCovarSum(const vector<Point2d>& p_hat, const vector<double>& w) {
		double mean = 0;
		for (int i=0; i<p_hat.size(); i++) {
			Vec2d p_hat_i = P2V(p_hat[i]);
			double prod = (p_hat_i.t() * p_hat_i).val[0];
			mean += (prod * w[i]);
		}
		return mean;
	}
	
	template<typename V>
	Matx22d GetWeightedCovarMat(const vector<Point_<V> >& p_hat, const vector<double>& w) {
		Matx22d covarmat(0.0);
		for (int i=0; i<p_hat.size(); i++) {
			Vec2d p_hat_i = P2V(p_hat[i]);
			covarmat = covarmat + p_hat_i * p_hat_i.t() * w[i];
		}
		covarmat = covarmat.inv();
		return covarmat;
	}
public:
	void Init(const vector<Point_<T> >& curve, const vector<int>& control_idx) {
		As.clear();
		mu_s.clear();
		mu_r.clear();
		control_pts.clear();
		deformed_control_pts.clear();
		m_curve.clear();
		m_original_curve.clear();
		m_w.clear();
		m_vmp.clear();
		
		m_curve = curve;
		m_original_curve = curve;
		
		Mat a_ = Mat(m_original_curve).reshape(1);
		PCA a_pca(a_,Mat(),CV_PCA_DATA_AS_ROW);
		m_original_curve_scale = sqrt(a_pca.eigenvalues.at<double>(0));
		
		
		vector<Point2d> p;
		for (int i=0; i<control_idx.size(); i++) {
			p.push_back(m_curve[control_idx[i]]);
			control_pts.push_back(m_curve[control_idx[i]]);
		}
		deformed_control_pts = control_pts;
		
		CalcWeights();
//		CalcGeodesicWeights(control_idx);
		
		A.create(m_curve.size(),control_idx.size());
		As.resize(m_curve.size(),vector<Matx22d>(control_idx.size()));
		m_vmp.resize(m_curve.size());
		
		for (int i=0; i<m_curve.size(); i++) {
			Vec2d v = P2V(m_curve[i]);
			
			Point2d p_star = GetWeightedMeanForPoint(i,p);
			vector<Point2d> p_hat = GetCurveAroundMean(p,p_star);
			
			//Affine - section 2.1
			for (int j=0; j<p.size(); j++) {
				Matx22d covmatx = GetWeightedCovarMat(p_hat,m_w[i]);
				Matx12d vmp = (v - P2V(p_star)).t();
				Matx21d p_hat_j_t = P2V(p_hat[j]);
				A(i,j) = (vmp * covmatx * p_hat_j_t).val[0] * m_w[i][j];
			}
			
			//Similarity - section 2.2
			double mu_s = GetWeightedCovarSum(p_hat, m_w[i]);
			for (int j=0; j<p.size(); j++) {
				//eqn (7)
				Matx22d lh(p_hat[j].x,p_hat[j].y,p_hat[j].y,-p_hat[j].x);
				m_vmp[i] = (v - P2V(p_star));
				Matx22d rh(m_vmp[i][0],m_vmp[i][1],m_vmp[i][1],-m_vmp[i][0]);
				
				As[i][j] = lh * rh.t() * (m_w[i][j] / mu_s);
			}
		}
	}
	
	void UpdateAffine() {
		vector<Point2d> q; ConvertCurve(deformed_control_pts, q);
		
		for (int i=0; i<m_curve.size(); i++) {
			Point2d q_star = GetWeightedMeanForPoint(i,q);
			vector<Point2d> q_hat = GetCurveAroundMean(q,q_star);

//			cout << A.row(i) << endl;
			Point2d newpoint(0,0);
			for (int j=0; j<q.size(); j++) {
				newpoint += q_hat[j] * A(i,j);
			}
			newpoint += q_star;
			
			m_curve[i] = Point_<T>(newpoint.x,newpoint.y);
		}
	}
	
	void UpdateSimilarity() {
		vector<Point2d> q; ConvertCurve(deformed_control_pts, q);
		
		for (int i=0; i<m_curve.size(); i++) {
			vector<double> w;
			Point2d q_star = GetWeightedMeanForPoint(i,q);
			vector<Point2d> q_hat = GetCurveAroundMean(q,q_star);
			
			Point2d newpoint(0,0);
			for (int j=0; j<q.size(); j++) {
				Matx22d as_i_j = As[i][j];
				Matx12d q_hat_j(q_hat[j].x,q_hat[j].y);
				Matx12d prod = q_hat_j * as_i_j;
				newpoint += Point2d(prod.val[0],prod.val[1]);
			}
			newpoint += q_star;
			
			m_curve[i] = Point_<T>(newpoint.x,newpoint.y);
		}
	}
	
	void UpdateRigid() {
		vector<Point2d> q; ConvertCurve(deformed_control_pts, q);
		
		for (int i=0; i<m_curve.size(); i++) {
			vector<double> w;
			Point2d q_star = GetWeightedMeanForPoint(i,q);
			vector<Point2d> q_hat = GetCurveAroundMean(q,q_star);
			
			Point2d newpoint(0,0);
			
			//calc sum_i(q_hat[i] * A[i]) 
			for (int j=0; j<q.size(); j++) {
				Matx22d as_i_j = As[i][j];
				Matx12d q_hat_j(q_hat[j].x,q_hat[j].y);
				Matx12d prod = q_hat_j * as_i_j;
				newpoint += Point2d(prod.val[0],prod.val[1]);
			}
			
			//eqn (8)
			double scale = norm(m_vmp[i]) / norm(newpoint);
			newpoint = newpoint * scale + q_star;
			
			m_curve[i] = Point_<T>(newpoint.x,newpoint.y);
		}		
	}
	
	const vector<Point_<T> >& GetControlPts() { return control_pts; }
	vector<Point_<T> >& GetDeformedControlPts() { return deformed_control_pts; }
	
	void Draw(Mat& img) {
//		img.setTo(0);
		{
			//draw small original
			vector<Point2d> tmp_curve;
			cv::transform(m_original_curve,tmp_curve,getRotationMatrix2D(Point2f(0,0),0,50/m_original_curve_scale));
//			Mat tmp_curve_m(tmp_curve); tmp_curve_m += Scalar(25,0);
			
			drawOpenCurve(img, tmp_curve, Scalar::all(255), 2);
		}			
		drawOpenCurve(img, m_curve, Scalar(0,0,255), 2);
		for (int i=0; i<control_pts.size(); i++) {
//			circle(img, control_pts[i], 3, Scalar(0,0,255), 1);
			circle(img, deformed_control_pts[i], 5, Scalar(0,255,255), 2);
		}
	}
};