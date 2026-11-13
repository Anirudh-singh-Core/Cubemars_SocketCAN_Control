"""Existing count-domain filters; no recommendations or automatic selection."""
import math

class LowPass:
    """Causal first-order filter using measured sample intervals, in count units."""

    def __init__(self, cutoff):
        if not math.isfinite(cutoff) or cutoff <= 0:
            raise ValueError('Cutoff must be finite and positive')
        self.cutoff = cutoff
        self.value = self.time = None

    def add(self, timestamp, raw):
        if self.value is None:
            self.value = float(raw)
        else:
            dt = timestamp - self.time
            if dt <= 0:
                raise ValueError('Sample timestamps must increase')
            alpha = -math.expm1(-2 * math.pi * self.cutoff * dt)
            self.value += alpha * (raw - self.value)
        self.time = timestamp
        return self.value

class AdaptiveEMA:
    """Heuristic noise-normalized EMA; calibrate at rest, then freeze noise scale.

    sigma = max(calibration population std, 0.5 count).
    r = abs(raw - previous_filtered) / (3*sigma).
    weight = max(0, r-1) / (1 + max(0, r-1)).
    fc = min_hz + weight*(max_hz-min_hz).
    alpha = 1-exp(-2*pi*fc*actual_dt). No deadband or output quantization.
    These settings are tunable engineering choices, not an optimal cutoff fit.
    """

    def __init__(self, min_hz=2.0, max_hz=50.0, calibration_s=2.0):
        if not all((math.isfinite(v) for v in (min_hz, max_hz, calibration_s))):
            raise ValueError('Filter settings must be finite')
        if not 0 < min_hz <= max_hz < 500 or calibration_s <= 0:
            raise ValueError('Require 0 < min Hz <= max Hz < 500 and calibration seconds > 0')
        self.min_hz, self.max_hz, self.calibration_s = (min_hz, max_hz, calibration_s)
        self.value = self.time = self.start = None
        self.n = 0
        self.mean = self.m2 = 0.0
        self.sigma = 0.5
        self.calibrated = False
        self.cutoff = min_hz
        self.alpha = 1.0

    def use_rest_variance(self, variance_counts_squared):
        """Use a completed external stationary acquisition; reset filter state."""
        if not math.isfinite(variance_counts_squared) or variance_counts_squared < 0:
            raise ValueError('Stationary variance must be finite and nonnegative')
        self.value = self.time = self.start = None
        self.sigma = max(0.5, math.sqrt(variance_counts_squared))
        self.calibrated = True
        return self

    def add(self, timestamp, raw):
        if not math.isfinite(timestamp) or not math.isfinite(raw):
            raise ValueError('Filter input must be finite')
        if self.time is not None and timestamp <= self.time:
            raise ValueError('Timestamps must increase')
        if self.start is None:
            self.start = timestamp
        if not self.calibrated:
            if timestamp - self.start < self.calibration_s or self.n < 2:
                self.n += 1
                delta = raw - self.mean
                self.mean += delta / self.n
                self.m2 += delta * (raw - self.mean)
                self.sigma = max(0.5, math.sqrt(max(0, self.m2 / self.n)))
            else:
                self.calibrated = True
        if self.value is None:
            self.value = float(raw)
        else:
            excess = max(0.0, abs(raw - self.value) / (3 * self.sigma) - 1) if self.calibrated else 0.0
            self.cutoff = self.min_hz + (self.max_hz - self.min_hz) * excess / (1 + excess)
            self.alpha = -math.expm1(-2 * math.pi * self.cutoff * (timestamp - self.time))
            self.value += self.alpha * (raw - self.value)
        self.time = timestamp
        return self.value

    def settings(self):
        return dict(method='noise-normalized adaptive EMA', min_cutoff_hz=self.min_hz, max_cutoff_hz=self.max_hz, calibration_seconds=self.calibration_s, calibration_samples=self.n, calibration_complete=self.calibrated, noise_sigma_counts=self.sigma, noise_floor_counts=0.5, threshold_sigma_multiplier=3, note='Heuristic settings; calibration assumes rest. Noise scale frozen after calibration; spikes can increase gain.')

class Kalman1D:
    """Scalar random-walk position filter, in raw CAN counts.

    Predict: P_prior = P + q_per_second*dt; x_prior = x.
    Update: K = P_prior/(P_prior+R); x += K*(measurement-x).
    Joseph covariance update preserves nonnegative covariance.
    R is estimated from the initial stationary calibration (sample variance),
    then frozen, with a configurable 0.25 count^2 default floor. Calibration
    outputs the running mean. Q is a tuning parameter, not learned from rest.
    """

    def __init__(self, q_per_second=1.0, calibration_s=2.0, r_floor=0.25):
        if not all((math.isfinite(v) for v in (q_per_second, calibration_s, r_floor))):
            raise ValueError('Settings must be finite')
        if q_per_second < 0 or calibration_s <= 0 or r_floor <= 0:
            raise ValueError('Require Q rate >= 0, calibration > 0 and R floor > 0')
        self.q_per_second, self.calibration_s, self.r_floor = (q_per_second, calibration_s, r_floor)
        self.value = self.time = self.start = None
        self.n = 0
        self.mean = self.m2 = 0.0
        self.r = self.p = r_floor
        self.gain = 1.0
        self.calibrated = False

    def use_rest_variance(self, variance_counts_squared):
        """Set R from external rest data; Q remains the caller's tuning choice."""
        if not math.isfinite(variance_counts_squared) or variance_counts_squared < 0:
            raise ValueError('Stationary variance must be finite and nonnegative')
        self.r = self.p = max(self.r_floor, variance_counts_squared)
        self.value = self.time = self.start = None
        self.calibrated = True
        return self

    def add(self, timestamp, raw):
        if not math.isfinite(timestamp) or not math.isfinite(raw):
            raise ValueError('Filter input must be finite')
        if self.time is not None and timestamp <= self.time:
            raise ValueError('Timestamps must increase')
        if self.start is None:
            self.start = timestamp
        dt = 0 if self.time is None else timestamp - self.time
        if self.calibrated and self.value is None:
            self.value = float(raw)
        elif not self.calibrated and (timestamp - self.start < self.calibration_s or self.n < 2):
            self.n += 1
            delta = raw - self.mean
            self.mean += delta / self.n
            self.m2 += delta * (raw - self.mean)
            self.r = max(self.r_floor, self.m2 / (self.n - 1)) if self.n > 1 else self.r_floor
            self.value = self.mean
            self.p = self.r / self.n
            self.gain = 1 / self.n
        else:
            self.calibrated = True
            prior = self.p + self.q_per_second * dt
            self.gain = prior / (prior + self.r)
            self.value += self.gain * (raw - self.value)
            self.p = (1 - self.gain) ** 2 * prior + self.gain ** 2 * self.r
        self.time = timestamp
        return self.value

    def settings(self):
        return dict(method='1-D random-walk position Kalman filter', process_noise_counts_squared_per_second=self.q_per_second, measurement_variance_counts_squared=self.r, measurement_variance_floor_counts_squared=self.r_floor, posterior_variance_counts_squared=self.p, calibration_seconds=self.calibration_s, calibration_samples=self.n, calibration_complete=self.calibrated, note='Q is a tunable assumption, not inferred from stationary noise. R assumes rest; correlated noise or motion can bias calibration. Covariance is model-based, not verified physical accuracy.')
