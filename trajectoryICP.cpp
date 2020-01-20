#include <iostream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>


using namespace std;

using namespace Sophus;
using namespace Eigen;

string compare_file = "compare.txt";

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

void ReadTrajectory(const string &path, TrajectoryType &esti,TrajectoryType &gt, VecVector3d &epts1, VecVector3d &gpts2 );

void pose_estimation_3d3d(const VecVector3d &pts1,
                          const VecVector3d &pts2,
                          Matrix3d &R, Vector3d &t);
int main(int argc, char **argv) {
    TrajectoryType estiPose, gtPose, correctPose;
    VecVector3d epts1, gpts2;
    ReadTrajectory(compare_file,estiPose, gtPose,epts1, gpts2);
    assert(!estiPose.empty() && !gtPose.empty());
    assert(estiPose.size() == gtPose.size());
    assert(!epts1.empty() && !gpts2.empty());
    assert(epts1.size() == gpts2.size());
    //DrawTrajectory(gtPose, estiPose);

  // compute rmse
    Matrix3d R;
    Vector3d t;
    pose_estimation_3d3d(gpts2,epts1,R,t);
    cout << "R=" << R << endl;
    cout << "t=" << t << endl;
    Sophus::SE3d T(R,t);

    for (int i = 0; i <estiPose.size() ; i++) {
        correctPose.push_back(T*estiPose[i]);
    }
    cout<<"correctPose"<<correctPose.size()<<endl;
    DrawTrajectory(gtPose, correctPose);


  return 0;
}

void ReadTrajectory(const string &path,
                    TrajectoryType &esti,TrajectoryType &gt,
                    VecVector3d &epts1, VecVector3d &gpts2 )
{
  ifstream fin(path);
  if (!fin) {
    cerr << "trajectory " << path << " not found." << endl;
  }

  while (!fin.eof()) {
    double time1, t1x, t1y, t1z, q1x, q1y, q1z, q1w, time2, t2x, t2y, t2z, q2x, q2y, q2z, q2w;
    fin >> time1 >> t1x >> t1y >> t1z >> q1x >> q1y >> q1z >> q1w>>
            time2 >> t2x >> t2y >> t2z >> q2x >> q2y >> q2z >> q2w;
    Eigen::Vector3d t1(t1x, t1y, t1z), t2(t2x, t2y, t2z);
    epts1.push_back(t1);
    gpts2.push_back(t2);
    Sophus::SE3d point1(Eigen::Quaterniond(q1w, q1x, q1y, q1z), t1),
            point2(Eigen::Quaterniond(q2w, q2x, q2y, q2z), t2);
      esti.push_back(point1);
      gt.push_back(point2);
  }
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));


  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glLineWidth(2);
    for (size_t i = 0; i < gt.size() - 1; i++) {
      glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
      glBegin(GL_LINES);
      auto p1 = gt[i], p2 = gt[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }

    for (size_t i = 0; i < esti.size() - 1; i++) {
      glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
      glBegin(GL_LINES);
      auto p1 = esti[i], p2 = esti[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }

}

void pose_estimation_3d3d(const VecVector3d &pts1,
                          const VecVector3d &pts2,
                          Matrix3d &R, Vector3d &t) {
    Vector3d p1, p2;     // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++) {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = p1 / N;
    p2 = p2 / N;
    VecVector3d q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++) {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++) {
        W += q1[i]* q2[i].transpose();
    }
    cout << "W=" << W << endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    cout << "U=" << U << endl;
    cout << "V=" << V << endl;

    R = U * (V.transpose());
    if (R.determinant() < 0) {
        R = -R;
    }
    t = p1 - R * p2;


}
