#ifndef SIMMY_BLDC_H
#define SIMMY_BLDC_H

/* Voltage-driven permanent-magnet models, SI units. Neither is a controller.
 * Linear independent winding inductances: no saliency, mutual inductance,
 * saturation, core/thermal/switching loss, parasitics or shorted-turn loops.
 * Angle is MECHANICAL radians, speed mechanical rad/s. Ke is phase-peak
 * back EMF per mechanical rad/s (also Nm/A per winding), NOT line RMS Kv.
 * Mechanical speed and midpoint rotor angle are held in each electrical
 * step. Backward Euler is first order, passive and resolves diode release
 * at grid accuracy. Refine dt with controller/PWM schedules fixed.
 */

typedef enum { SIMMY_WYE = 0, SIMMY_DELTA = 1 } SimmyWindingConnection;
typedef enum { SIMMY_PM_SINE = 0, SIMMY_PM_TRAPEZOID = 1 } SimmyPmWaveform;

typedef struct {
  int polePairs;
  SimmyWindingConnection connection;
  SimmyPmWaveform waveform;
  double resistance[3], inductance[3], backEmfConstant[3];
  int openWinding[3]; /* static open circuit; may open ONLY at zero current.
                      * No arc/capacitance model for interrupting inductive current. */
  double electricalOffset; /* winding A magnetic axis offset, electrical rad */
  double thirdHarmonic;    /* optional additional -Ke*h*sin(3*angle) */
} SimmyBldcParams;

typedef struct {
  double windingHeatJ[3], switchHeatJ[6], diodeHeatJ[6];
  double inputJ, mechanicalWorkJ, numericalLossJ;
} SimmyBldcEnergy;

typedef struct {
  /* Wye: A/B/C -> floating neutral. Delta: A->B, B->C, C->A.
   * Delta winding currents are NOT terminal currents; retain circulation. */
  double current[3];
  int diodeMode[3]; /* solver warm start, not a requested switch state */
  SimmyBldcEnergy energy;
} SimmyBldcState;

typedef struct {
  double busVoltage, diodeDrop;
  /* Indices 2*leg = low, 2*leg+1 = high. R>=0; zero is ideal ON.
   * Open channel retains its body diode. No shorted-device/shoot-through
   * model: that needs DC-link impedance/protection, not motor L alone. */
  double onResistance[6];
  int openSwitch[6];
} SimmyBridgeParams;

typedef struct {
  double terminalCurrent[3], terminalVoltage[3];
  double windingVoltage[3]; /* branch drop; includes gap voltage if open */
  double windingLossW[3], switchLossW[6], diodeLossW[6];
  double torque, magneticEnergy, inputPower, mechanicalPower, numericalLossJ;
} SimmyBldcOutput;

int SimmyBldcParamsValid(const SimmyBldcParams *params);
int SimmyBridgeParamsValid(const SimmyBridgeParams *params);
void SimmyBldcInit(SimmyBldcState *state);
/* Ideal imposed terminal potentials, floating wye neutral / closed delta.
 * Step outputs use end currents and held midpoint EMF/angle. Input power
 * includes regeneration. Returns 0 on invalid input/no circuit solution,
 * leaves state AND output unchanged. Wye initial currents must sum to zero. */
int SimmyBldcVoltageStep(SimmyBldcState *state, const SimmyBldcParams *params,
                         const double terminalVoltage[3], double angle, double speed,
                         double dt, SimmyBldcOutput *output);
/* command[leg]: -1 low ON, 0 both OFF, +1 high ON. Fixed ideal DC bus can
 * absorb regeneration. Floating terminals are NOT set to zero voltage:
 * an active-set circuit solve determines diode clamps or zero terminal
 * current. No current clipping/instant clearing, or imposed mechanical state.
 * Loss ledger: input - mechanical work - heat - change in magnetic energy
 * = numericalLossJ. Numerical damping is NOT reported as physical heat. */
int SimmyBldcBridgeStep(SimmyBldcState *state, const SimmyBldcParams *params,
                        const SimmyBridgeParams *bridge, const int command[3],
                        double angle, double speed, double dt, SimmyBldcOutput *output);
double SimmyBldcHeatJ(const SimmyBldcEnergy *energy);
/* Six-sector commutator; angle is sensed/calibrated electrical angle,
 * not a motor-state pointer. Offset may represent timing/table errors.
 * Delta sine winding axes need -pi/6 relative to wye with identical coils.
 * Finite angles/non-null output required. Produces high/low/floating
 * decisions, not currents or torque. */
void SimmyBldcSixStep(double sensedElectricalAngle, double advance,
                      int sectorOffset, int command[3]);

/* Reduced healthy balanced model, stationary amplitude-invariant alpha/
 * beta current. Sine AND trapezoidal wye reductions are exact with equal
 * R/L and imposed terminal voltages. Parameters are floating-wye values.
 * For equivalent delta branches: Rdelta=3R, Ldelta=3L, Kedelta=sqrt(3)Ke,
 * delta winding offset = reduced offset + pi/6; no circulating mode.
 * That delta mapping is SINE ONLY. Trapezoidal delta can circulate and
 * cannot generally be reduced to this two-state model.
 * With identical physical coils, simply rewiring is NOT this equivalence. */
typedef struct {
  int polePairs;
  SimmyPmWaveform waveform;
  double resistance, inductance, backEmfConstant, electricalOffset;
} SimmyPmParams;
typedef struct {
  double current[2];
  double windingHeatJ, inputJ, mechanicalWorkJ, numericalLossJ;
} SimmyPmState;
typedef struct {
  double torque, magneticEnergy, windingLossW, inputPower, numericalLossJ;
} SimmyPmOutput;
int SimmyPmParamsValid(const SimmyPmParams *params);
void SimmyPmInit(SimmyPmState *state);
int SimmyPmStep(SimmyPmState *state, const SimmyPmParams *params,
                 const double voltage[2], double angle, double speed, double dt,
                 SimmyPmOutput *output);

/* Brushed DC: a real brush/commutator pair keeps the winding facing the
 * magnet always energized the same way, so -- unlike every other motor
 * here -- torque and back EMF don't depend on rotor angle at all. That
 * collapses the model to Kirchhoff's voltage law on a single winding.
 * backEmfConstant is this motor's Ke and Kt at once: Nm/A == V*s/rad in
 * consistent SI units, the same invariant the other PM models use. */
typedef struct {
  double resistance;      /* ohms, >=0 */
  double inductance;      /* H, >0 */
  double backEmfConstant; /* Nm/A == V*s/rad */
} SimmyDcParams;
typedef struct {
  double current;                                                /* A */
  double windingHeatJ, inputJ, mechanicalWorkJ, numericalLossJ;   /* J, cumulative */
} SimmyDcState;
typedef struct {
  double torque, magneticEnergy, windingLossW, inputPower, numericalLossJ;
} SimmyDcOutput;
int SimmyDcParamsValid(const SimmyDcParams *params);
void SimmyDcInit(SimmyDcState *state);
/* Backward Euler, held speed over dt>0. Returns 0 on invalid input,
 * leaves state AND output unchanged -- same convention as SimmyPmStep. */
int SimmyDcStep(SimmyDcState *state, const SimmyDcParams *params,
                double voltage, double speed, double dt, SimmyDcOutput *output);

#endif
