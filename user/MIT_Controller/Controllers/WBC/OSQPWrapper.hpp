#ifndef WBC_OSQP_WRAPPER_HPP
#define WBC_OSQP_WRAPPER_HPP

// OSQP 工作区管理（供 QPWBC 使用）
// 设计：
//   - setup: 维度或稀疏结构变化时调用（cleanup 旧工作区 + 重新分配）
//   - update: 维度与稀疏结构不变时调用（osqp_update_P_A + lin_cost + bounds + warm_start_x）
//   - solve: osqp_solve
//   - 析构: osqp_cleanup（修复 SparseCMPC 注释掉 cleanup 的泄漏）
//
// 输入约定：P/A 为列主序 dense（Eigen 默认），P 仅取上三角
// 内部按 (列, 行) 顺序压缩为 CSC，与 osqp_setup 后 work->data->P->x / A->x 顺序一致，
// 因此 update 时直接按存储的 (row, col) 重读值即可

#include <vector>
#include "osqp.h"

class OSQPWrapper {
 public:
  OSQPWrapper() { _settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings)); }
  ~OSQPWrapper() {
    cleanup();
    if (_settings) c_free(_settings);
  }

  OSQPWrapper(const OSQPWrapper&) = delete;
  OSQPWrapper& operator=(const OSQPWrapper&) = delete;

  // 完整 setup
  //   n = 变量数, m = 约束数
  //   P: n×n 列主序 dense（仅上三角非零被采用）
  //   A: m×n 列主序 dense
  //   q: n, l/u: m
  bool setup(int n, int m, const c_float* P, const c_float* A,
             const c_float* q, const c_float* l, const c_float* u) {
    cleanup();
    _n = n;
    _m = m;

    buildCSC(P, A);

    csc P_csc;
    P_csc.m = n; P_csc.n = n; P_csc.nz = -1;
    P_csc.nzmax = (c_int)_P_val.size();
    P_csc.x = _P_val.data();
    P_csc.i = _P_row.data();
    P_csc.p = _P_col_ptr.data();

    csc A_csc;
    A_csc.m = m; A_csc.n = n; A_csc.nz = -1;
    A_csc.nzmax = (c_int)_A_val.size();
    A_csc.x = _A_val.data();
    A_csc.i = _A_row.data();
    A_csc.p = _A_col_ptr.data();

    OSQPData data;
    data.n = n;
    data.m = m;
    data.P = &P_csc;
    data.A = &A_csc;
    data.q = const_cast<c_float*>(q);
    data.l = const_cast<c_float*>(l);
    data.u = const_cast<c_float*>(u);

    osqp_set_default_settings(_settings);
    _settings->eps_abs = 1e-6;
    _settings->eps_rel = 1e-6;
    _settings->max_iter = 1000;
    _settings->verbose = 0;
    _settings->warm_start = 1;

    OSQPWorkspace* wk = osqp_setup(&data, _settings);
    if (!wk) return false;
    _workspace = wk;
    return true;
  }

  // 更新值（稀疏结构与 setup 时一致）
  bool update(const c_float* P, const c_float* A, const c_float* q,
              const c_float* l, const c_float* u, const c_float* x_warm) {
    if (!_workspace) return false;

    // 按 setup 时记录的 (row, col) 重读值，顺序与 work->data->P->x / A->x 一致
    for (size_t k = 0; k < _P_row.size(); ++k) {
      _P_val[k] = P[_P_row[k] + _P_col_idx[k] * _n];
    }
    for (size_t k = 0; k < _A_row.size(); ++k) {
      _A_val[k] = A[_A_row[k] + _A_col_idx[k] * _n];
    }

    c_int ret = 0;
    ret |= osqp_update_P_A(_workspace, _P_val.data(), OSQP_NULL,
                           (c_int)_P_val.size(), _A_val.data(), OSQP_NULL,
                           (c_int)_A_val.size());
    ret |= osqp_update_lin_cost(_workspace, q);
    ret |= osqp_update_bounds(_workspace, l, u);
    if (x_warm) osqp_warm_start_x(_workspace, x_warm);
    return ret == 0;
  }

  bool solve() {
    if (!_workspace) return false;
    _status = osqp_solve(_workspace);
    return _status == 0;
  }

  const c_float* solutionX() const {
    return _workspace ? _workspace->solution->x : nullptr;
  }

  bool isInitialized() const { return _workspace != nullptr; }
  int n() const { return _n; }
  int m() const { return _m; }
  c_int status() const { return _status; }

 private:
  OSQPWorkspace* _workspace = nullptr;
  OSQPSettings* _settings = nullptr;
  int _n = 0, _m = 0;
  c_int _status = 0;

  // CSC 缓冲：
  //   _P_val/_A_val: 非零值（与 workspace 内部顺序一致）
  //   _P_row/_A_row: 逐元素行索引（同时作 CSC rowIdx）
  //   _P_col_idx/_A_col_idx: 逐元素列索引（仅 update 回填用）
  //   _P_col_ptr/_A_col_ptr: CSC 列指针（长度 n+1）
  std::vector<c_float> _P_val, _A_val;
  std::vector<c_int> _P_row, _P_col_idx, _P_col_ptr;
  std::vector<c_int> _A_row, _A_col_idx, _A_col_ptr;

  void buildCSC(const c_float* P, const c_float* A) {
    _P_val.clear(); _P_row.clear(); _P_col_idx.clear();
    _P_col_ptr.assign(1, 0);
    for (int j = 0; j < _n; ++j) {
      for (int i = 0; i <= j; ++i) {
        c_float v = P[i + j * _n];
        if (v != 0.0) {
          _P_val.push_back(v);
          _P_row.push_back(i);
          _P_col_idx.push_back(j);
        }
      }
      _P_col_ptr.push_back((c_int)_P_val.size());
    }

    _A_val.clear(); _A_row.clear(); _A_col_idx.clear();
    _A_col_ptr.assign(1, 0);
    for (int j = 0; j < _n; ++j) {
      for (int i = 0; i < _m; ++i) {
        c_float v = A[i + j * _n];
        if (v != 0.0) {
          _A_val.push_back(v);
          _A_row.push_back(i);
          _A_col_idx.push_back(j);
        }
      }
      _A_col_ptr.push_back((c_int)_A_val.size());
    }
  }

  void cleanup() {
    if (_workspace) {
      osqp_cleanup(_workspace);
      _workspace = nullptr;
    }
  }
};

#endif  // WBC_OSQP_WRAPPER_HPP
