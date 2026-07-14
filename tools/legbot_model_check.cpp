#include <Controllers/LegController.h>
#include <Dynamics/FloatingBaseModel.h>
#include <Dynamics/LegBot.h>

#include <array>
#include <cstdio>

namespace {

constexpr std::array<const char*, 4> kLegNames = {"FR", "FL", "RR", "RL"};

template <typename T>
void printVec3(const char* name, const Vec3<T>& v) {
  std::printf("%s = [% .6f % .6f % .6f]\n", name, (double)v[0],
              (double)v[1], (double)v[2]);
}

template <typename T>
void printMat3(const char* name, const Mat3<T>& m) {
  std::printf("%s =\n", name);
  for (int r = 0; r < 3; ++r) {
    std::printf("  [% .6f % .6f % .6f]\n", (double)m(r, 0),
                (double)m(r, 1), (double)m(r, 2));
  }
}

template <typename T>
void printD3MatJointBlock(const char* name, const D3Mat<T>& m, int col0) {
  std::printf("%s cols[%d:%d] =\n", name, col0, col0 + 2);
  for (int r = 0; r < 3; ++r) {
    std::printf("  [% .6f % .6f % .6f]\n", (double)m(r, col0),
                (double)m(r, col0 + 1), (double)m(r, col0 + 2));
  }
}

}  // namespace

int main() {
  using T = double;

  Quadruped<T> legbot = buildLegBot<T>();
  FloatingBaseModel<T> model = legbot.buildModel();

  FBModelState<T> state;
  state.bodyOrientation = Quat<T>(1, 0, 0, 0);
  state.bodyPosition = Vec3<T>(0, 0, 0);
  state.bodyVelocity = SVec<T>::Zero();
  state.q = DVec<T>::Zero(model._nDof - 6);
  state.qd = DVec<T>::Zero(model._nDof - 6);

  std::array<Vec3<T>, 4> qLegs = {
      Vec3<T>(-0.04, 0.875, -1.667),
      Vec3<T>(0.04, 0.875, -1.667),
      Vec3<T>(-0.04, 0.875, -1.667),
      Vec3<T>(0.04, 0.875, -1.667),
  };

  for (int leg = 0; leg < 4; ++leg) {
    state.q.segment<3>(3 * leg) = qLegs[leg];
  }

  model.setState(state);
  model.forwardKinematics();
  model.contactJacobians();

  std::printf("LegBot model consistency check\n");
  std::printf("q order per leg: abad, thigh, calf\n");
  std::printf("Expected from MuJoCo numeric check: Jx_thigh < 0, Jx_calf < 0\n");
  std::printf("FloatingBaseModel _nDof = %zu, foot contacts = %zu\n\n",
              model._nDof, model._footIndicesGC.size());

  for (int leg = 0; leg < 4; ++leg) {
    Mat3<T> legJ = Mat3<T>::Zero();
    Vec3<T> legP = Vec3<T>::Zero();
    Vec3<T> q = qLegs[leg];
    computeLegJacobianAndPosition<T>(legbot, q, &legJ, &legP, leg);

    const size_t gc = model._footIndicesGC.at(leg);
    const int jointCol0 = 6 + 3 * leg;
    const D3Mat<T>& jc = model._Jc.at(gc);

    std::printf("------------------------------------------------------------\n");
    std::printf("leg %d (%s), foot GC index %zu, GC parent body %zu\n", leg,
                kLegNames[leg], gc, model._gcParent.at(gc));
    std::printf("q = [% .6f % .6f % .6f]\n", (double)q[0], (double)q[1],
                (double)q[2]);

    printVec3("LegController pFoot", legP);
    printVec3("FloatingBaseModel pGC", model._pGC.at(gc));
    std::printf("\n");

    printMat3("LegController J", legJ);
    printD3MatJointBlock("FloatingBaseModel Jc joint block", jc, jointCol0);
    std::printf("\n");

    std::printf("summary:\n");
    std::printf("  LegController    Jx_thigh=% .6f Jx_calf=% .6f "
                "Jz_thigh=% .6f Jz_calf=% .6f\n",
                (double)legJ(0, 1), (double)legJ(0, 2),
                (double)legJ(2, 1), (double)legJ(2, 2));
    std::printf("  FloatingBaseModel Jx_thigh=% .6f Jx_calf=% .6f "
                "Jz_thigh=% .6f Jz_calf=% .6f\n\n",
                (double)jc(0, jointCol0 + 1),
                (double)jc(0, jointCol0 + 2),
                (double)jc(2, jointCol0 + 1),
                (double)jc(2, jointCol0 + 2));
  }

  return 0;
}
