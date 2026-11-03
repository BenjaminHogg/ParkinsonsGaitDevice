#pragma once

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "fog_model.h"

enum FogEvent {
  FOG_NONE = 0,
  FOG_START = 1,
  FOG_END = 2
};

class FogDetector {
public:
  void begin(float imuHz, float threshold = 0.0f, int confirm = 2) {
    if (imuHz <= 0.0f) imuHz = FOG_FS;

    _step = imuHz / FOG_FS;
    _thr = threshold;
    _confirm = (confirm < 1) ? 1 : confirm;

    for (int n = 0; n < FOG_WIN; n++) {
      _w[n] = 0.5f - 0.5f *
        cosf(2.0f * PI * n / (FOG_WIN - 1));

      _cos[n] = cosf(2.0f * PI * n / FOG_WIN);
      _sin[n] = sinf(2.0f * PI * n / FOG_WIN);
    }
  }

  FogEvent update(float ax, float ay, float az) {
    float cur[3] = {ax, ay, az};
    _ev = FOG_NONE;

    if (_idx == 0) {
      memcpy(_prev, cur, sizeof(cur));
      _push(cur);
      _nextPos = _step;
      _idx = 1;
      return _ev;
    }

    while (_nextPos <= (float)_idx) {
      float frac = _nextPos - (float)(_idx - 1);
      float o[3];

      for (int a = 0; a < 3; a++) {
        o[a] = _prev[a] + frac * (cur[a] - _prev[a]);
      }

      _push(o);
      _nextPos += _step;
    }

    memcpy(_prev, cur, sizeof(cur));
    _idx++;

    return _ev;
  }

  bool freezing() const {
    return _freezing;
  }

  float score() const {
    return _z;
  }

  float fFreeze = 0.0f;
  float fLoco = 0.0f;

private:
  float _w[FOG_WIN];
  float _cos[FOG_WIN];
  float _sin[FOG_WIN];
  float _buf[3][FOG_WIN];

  float _prev[3] = {0, 0, 0};
  float _step = 1.0f;
  float _nextPos = 0.0f;
  float _thr = 0.0f;
  float _z = 0.0f;

  uint32_t _idx = 0;
  int _head = 0;
  int _filled = 0;
  int _sinceHop = 0;
  int _posRun = 0;
  int _negRun = 0;
  int _confirm = 2;

  bool _freezing = false;
  FogEvent _ev = FOG_NONE;

  void _push(const float* o) {
    for (int a = 0; a < 3; a++) {
      _buf[a][_head] = o[a];
    }

    _head = (_head + 1) % FOG_WIN;

    if (_filled < FOG_WIN) {
      _filled++;
    }

    if (++_sinceHop >= FOG_HOP && _filled == FOG_WIN) {
      _sinceHop = 0;
      _evaluate();
    }
  }

  void _evaluate() {
    float Pf = 0.0f;
    float Pl = 0.0f;
    float x[FOG_WIN];

    for (int a = 0; a < 3; a++) {
      float m = 0.0f;

      for (int n = 0; n < FOG_WIN; n++) {
        x[n] = _buf[a][(_head + n) % FOG_WIN];
        m += x[n];
      }

      m /= FOG_WIN;

      for (int n = 0; n < FOG_WIN; n++) {
        x[n] = (x[n] - m) * _w[n];
      }

      for (int k = 2; k <= 32; k++) {
        float re = 0.0f;
        float im = 0.0f;

        for (int n = 0; n < FOG_WIN; n++) {
          int i = (k * n) & (FOG_WIN - 1);
          re += x[n] * _cos[i];
          im -= x[n] * _sin[i];
        }

        float p = (re * re + im * im) / FOG_WIN;

        if (k < 12) {
          Pl += p;
        } else {
          Pf += p;
        }
      }
    }

    fFreeze = log10f(Pf + 1e-6f);
    fLoco = log10f(Pl + 1e-6f);

    _z = FOG_W_FREEZE * fFreeze
       + FOG_W_LOCO * fLoco
       + FOG_BIAS;

    if (_z > _thr) {
      _posRun++;
      _negRun = 0;
    } else {
      _negRun++;
      _posRun = 0;
    }

    if (!_freezing && _posRun >= _confirm) {
      _freezing = true;
      _ev = FOG_START;
    }

    if (_freezing && _negRun >= _confirm) {
      _freezing = false;
      _ev = FOG_END;
    }
  }
};
