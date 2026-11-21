#include "simmy_bldc.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846

int SimmyBldcParamsValid(const SimmyBldcParams *p) {
  if (!p || p->polePairs <= 0 || (p->connection != SIMMY_WYE && p->connection != SIMMY_DELTA) ||
      (p->waveform != SIMMY_PM_SINE && p->waveform != SIMMY_PM_TRAPEZOID) ||
      !isfinite(p->electricalOffset) || !isfinite(p->thirdHarmonic)) return 0;
  for (int k=0;k<3;k++)
    if (!isfinite(p->resistance[k]) || p->resistance[k]<0 ||
        !isfinite(p->inductance[k]) || p->inductance[k]<=0 ||
        !isfinite(p->backEmfConstant[k]) || p->backEmfConstant[k]<0) return 0;
  return 1;
}
int SimmyBridgeParamsValid(const SimmyBridgeParams *p) {
  if (!p || !isfinite(p->busVoltage) || p->busVoltage<=0 ||
      !isfinite(p->diodeDrop) || p->diodeDrop<0) return 0;
  for (int k=0;k<6;k++) if (!isfinite(p->onResistance[k]) || p->onResistance[k]<0) return 0;
  return 1;
}
void SimmyBldcInit(SimmyBldcState *s) { memset(s,0,sizeof(*s)); }
double SimmyBldcHeatJ(const SimmyBldcEnergy *e) {
  double result=0;
  for (int k=0;k<3;k++) result+=e->windingHeatJ[k];
  for (int k=0;k<6;k++) result+=e->switchHeatJ[k]+e->diodeHeatJ[k];
  return result;
}

static double TrapSin(double x) {
  x=remainder(x,2*PI);
  double a=fabs(x), y=a<PI/6 ? 6*a/PI : (a<=5*PI/6 ? 1 : 6*(PI-a)/PI);
  return copysign(y,x);
}
static void Coefficients(const SimmyBldcParams *p, double angle, double k[3]) {
  double theta=remainder(p->polePairs*angle+p->electricalOffset,2*PI);
  for (int j=0;j<3;j++) {
    double x=theta-2*PI*j/3;
    double wave=p->waveform==SIMMY_PM_SINE ? sin(x) : TrapSin(x);
    k[j]=-p->backEmfConstant[j]*(wave+p->thirdHarmonic*sin(3*x));
  }
}
static void Incidence(SimmyWindingConnection connection, double b[3][3]) {
  memset(b,0,9*sizeof(double));
  for (int j=0;j<3;j++) {
    b[j][j]=1;
    if (connection==SIMMY_DELTA) b[(j+1)%3][j]=-1;
  }
}
/* Small dense solve with partial pivoting. All coefficients are finite
 * validated inputs; no inverse, regularization or hidden leakage resistor. */
static int Solve(double a[7][8], int n, double x[7]) {
  for (int k=0;k<n;k++) {
    int pivot=k;
    for (int r=k+1;r<n;r++) if (fabs(a[r][k])>fabs(a[pivot][k])) pivot=r;
    if (fabs(a[pivot][k])<1e-14) return 0;
    if (pivot!=k) for (int c=k;c<=n;c++) {
      double t=a[k][c]; a[k][c]=a[pivot][c]; a[pivot][c]=t;
    }
    for (int r=k+1;r<n;r++) {
      double ratio=a[r][k]/a[k][k];
      for (int c=k+1;c<=n;c++) a[r][c]-=ratio*a[k][c];
    }
  }
  for (int r=n-1;r>=0;r--) {
    double value=a[r][n];
    for (int c=r+1;c<n;c++) value-=a[r][c]*x[c];
    x[r]=value/a[r][r];
    if (!isfinite(x[r])) return 0;
  }
  return 1;
}

static int Root(const int parent[4], int node) {
  while (parent[node]!=node) node=parent[node];
  return node;
}

static int Step(SimmyBldcState *s, const SimmyBldcParams *p,
                 const SimmyBridgeParams *bridge, const int command[3], const double imposed[3],
                 double angle, double speed, double dt, SimmyBldcOutput *output) {
  if (!s || !output || !SimmyBldcParamsValid(p) || !isfinite(angle) ||
      !isfinite(speed) || !isfinite(dt) || dt<=0) return 0;
  if (bridge && (!command || !SimmyBridgeParamsValid(bridge))) return 0;
  if (!bridge && !imposed) return 0;
  double sum=0, norm=0;
  for (int j=0;j<3;j++) {
    if (!isfinite(s->current[j]) || (p->openWinding[j] && s->current[j]!=0) ||
        (bridge && (command[j]<-1 || command[j]>1)) ||
        (!bridge && !isfinite(imposed[j]))) return 0;
    sum+=s->current[j]; norm+=fabs(s->current[j]);
  }
  if (p->connection==SIMMY_WYE && fabs(sum)>1e-9*(1+norm)) return 0;
  double b[3][3], coefficient[3];
  Incidence(p->connection,b);
  Coefficients(p,angle+.5*speed*dt,coefficient);
  int n=p->connection==SIMMY_WYE ? 7 : 6;
  int parent[4]={0,1,2,3};
  for (int j=0;j<3;j++) if (!p->openWinding[j]) {
    int end=p->connection==SIMMY_WYE ? 3 : (j+1)%3;
    parent[Root(parent,j)]=Root(parent,end);
  }
  int active[3]={-1,-1,-1};
  if (bridge) for (int j=0;j<3;j++) {
    int channel=command[j] ? 2*j+(command[j]>0) : -1;
    if (channel>=0 && !bridge->openSwitch[channel]) active[j]=channel;
  }
  int warm=0;
  for (int j=2;j>=0;j--) warm=3*warm+(s->diodeMode[j]>=0 && s->diodeMode[j]<=2 ? s->diodeMode[j] : 0);
  /* Try last active set first, then exhaust the 3^3 legal diode choices.
   * For mode 0: ON resistor/source or open terminal with Iline_next=0.
   * Mode 1/2: lower/upper diode clamps plus any commanded ON resistor. */
  for (int attempt=-1;attempt<(bridge ? 27 : 0);attempt++) {
    if (attempt==warm) continue;
    int code=attempt<0 ? warm : attempt, mode[3], invalid=0;
    int anchored[4]={0}, gauge[4]={-1,-1,-1,-1};
    double a[7][8]={{0}}, x[7]={0};
    for (int j=0;j<3;j++) {
      mode[j]=bridge ? code%3 : 0; code/=3;
      if (bridge && active[j]>=0 && bridge->onResistance[active[j]]==0 && mode[j]) invalid=1;
      if (!bridge || mode[j] || active[j]>=0) anchored[Root(parent,j)]=1;
      if (p->openWinding[j]) a[j][j]=1;
      else {
        a[j][j]=p->inductance[j]/dt+p->resistance[j];
        a[j][n]=p->inductance[j]/dt*s->current[j]-coefficient[j]*speed;
        for (int node=0;node<3;node++) a[j][3+node]=-b[node][j];
        if (n==7) a[j][6]=1;
      }
      if (n==7) a[6][j]=1;
      int row=3+j;
      if (!bridge) { a[row][3+j]=1; a[row][n]=imposed[j]; }
      else if (mode[j]) {
        a[row][3+j]=1;
        a[row][n]=mode[j]==1 ? -bridge->diodeDrop : bridge->busVoltage+bridge->diodeDrop;
      } else if (active[j]>=0 && bridge->onResistance[active[j]]==0) {
        a[row][3+j]=1; a[row][n]=(active[j]%2) ? bridge->busVoltage : 0;
      } else {
        for (int coil=0;coil<3;coil++) a[row][coil]=b[j][coil];
        if (active[j]>=0) {
          double conductance=1/bridge->onResistance[active[j]];
          a[row][3+j]=conductance;
          a[row][n]=(active[j]%2) ? bridge->busVoltage*conductance : 0;
        }
      }
    }
    if (invalid) continue;
    /* One gauge per electrically floating connected component. This also
     * handles isolated terminals after an open winding, without artificial
     * shunt resistors or current/energy clearing. */
    for (int node=0;node<(n==7 ? 4 : 3);node++) {
      int root=Root(parent,node);
      if (!anchored[root] && gauge[root]<0) {
        gauge[root]=node;
        int row=node==3 ? 6 : 3+node;
        memset(a[row],0,sizeof(a[row])); a[row][3+node]=1;
      }
    }
    if (!Solve(a,n,x)) continue;
    if (bridge) for (int root=0;root<4;root++) if (gauge[root]>=0 && gauge[root]<3) {
      double lo=INFINITY, hi=-INFINITY;
      for (int node=0;node<3;node++) if (Root(parent,node)==root) {
        lo=fmin(lo,x[3+node]); hi=fmax(hi,x[3+node]);
      }
      double shift=.5*(bridge->busVoltage-lo-hi);
      for (int node=0;node<(n==7 ? 4 : 3);node++) if (Root(parent,node)==root) x[3+node]+=shift;
    }
    SimmyBldcOutput o={0};
    for (int node=0;node<3;node++) {
      o.terminalVoltage[node]=x[3+node];
      for (int coil=0;coil<3;coil++) o.terminalCurrent[node]+=b[node][coil]*x[coil];
      if (bridge) {
        double v=x[3+node], line=o.terminalCurrent[node], channelCurrent=0;
        int channel=active[node];
        if (channel>=0) {
          double r=bridge->onResistance[channel];
          channelCurrent=r>0 ? ((channel%2 ? bridge->busVoltage : 0)-v)/r : line;
          o.switchLossW[channel]=r*channelCurrent*channelCurrent;
          if (channel%2) o.inputPower+=bridge->busVoltage*channelCurrent;
        }
        double diodeCurrent=line-channelCurrent;
        if (!mode[node]) {
          if (v < -bridge->diodeDrop-1e-8 || v > bridge->busVoltage+bridge->diodeDrop+1e-8) invalid=1;
        } else {
          if ((mode[node]==1 && diodeCurrent < -1e-8) || (mode[node]==2 && diodeCurrent > 1e-8)) invalid=1;
          o.diodeLossW[2*node+(mode[node]==2)]=bridge->diodeDrop*fabs(diodeCurrent);
          if (mode[node]==2) o.inputPower+=bridge->busVoltage*diodeCurrent;
        }
      } else o.inputPower+=x[3+node]*o.terminalCurrent[node];
    }
    if (invalid) continue;
    for (int j=0;j<3;j++) {
      double change=x[j]-s->current[j];
      o.torque+=coefficient[j]*x[j];
      o.magneticEnergy+=.5*p->inductance[j]*x[j]*x[j];
      o.windingLossW[j]=p->resistance[j]*x[j]*x[j];
      o.numericalLossJ+=.5*p->inductance[j]*change*change;
      for (int node=0;node<3;node++) o.windingVoltage[j]+=b[node][j]*x[3+node];
      if (n==7) o.windingVoltage[j]-=x[6];
    }
    o.mechanicalPower=o.torque*speed;
    if (!isfinite(o.torque) || !isfinite(o.magneticEnergy) || !isfinite(o.inputPower) ||
        !isfinite(o.mechanicalPower) || !isfinite(o.numericalLossJ)) return 0;
    SimmyBldcEnergy energy=s->energy;
    for (int j=0;j<3;j++) {
      energy.windingHeatJ[j]+=dt*o.windingLossW[j];
      if (!isfinite(o.terminalCurrent[j]) || !isfinite(o.terminalVoltage[j]) ||
          !isfinite(o.windingVoltage[j]) || !isfinite(o.windingLossW[j]) ||
          !isfinite(energy.windingHeatJ[j])) return 0;
    }
    for (int j=0;j<6;j++) {
      energy.switchHeatJ[j]+=dt*o.switchLossW[j];
      energy.diodeHeatJ[j]+=dt*o.diodeLossW[j];
      if (!isfinite(o.switchLossW[j]) || !isfinite(o.diodeLossW[j]) ||
          !isfinite(energy.switchHeatJ[j]) || !isfinite(energy.diodeHeatJ[j])) return 0;
    }
    energy.inputJ+=dt*o.inputPower;
    energy.mechanicalWorkJ+=dt*o.mechanicalPower;
    energy.numericalLossJ+=o.numericalLossJ;
    if (!isfinite(energy.inputJ) || !isfinite(energy.mechanicalWorkJ) || !isfinite(energy.numericalLossJ)) return 0;
    /* Commit only after the circuit, outputs AND cumulative ledger pass. */
    for (int j=0;j<3;j++) { s->current[j]=x[j]; s->diodeMode[j]=mode[j]; }
    s->energy=energy;
    *output=o;
    return 1;
  }
  return 0;
}
int SimmyBldcVoltageStep(SimmyBldcState *s, const SimmyBldcParams *p,
                         const double v[3], double angle, double speed, double dt, SimmyBldcOutput *o) {
  return Step(s,p,0,0,v,angle,speed,dt,o);
}
int SimmyBldcBridgeStep(SimmyBldcState *s, const SimmyBldcParams *p,
                        const SimmyBridgeParams *bridge, const int command[3],
                        double angle, double speed, double dt, SimmyBldcOutput *o) {
  if (!bridge) return 0;
  return Step(s,p,bridge,command,0,angle,speed,dt,o);
}
void SimmyBldcSixStep(double angle, double advance, int sectorOffset, int command[3]) {
  double theta=remainder(angle+advance+remainder((double)sectorOffset,6)*PI/3,2*PI);
  int high=0,low=0; double hi=-sin(theta),lo=hi;
  for (int j=0;j<3;j++) {
    command[j]=0; double k=-sin(theta-2*PI*j/3);
    if (k>hi) { hi=k; high=j; } if (k<lo) { lo=k; low=j; }
  }
  command[high]=1; command[low]=-1;
}

int SimmyPmParamsValid(const SimmyPmParams *p) {
  return p && p->polePairs>0 && (p->waveform==SIMMY_PM_SINE || p->waveform==SIMMY_PM_TRAPEZOID) &&
      isfinite(p->resistance) && p->resistance>=0 &&
      isfinite(p->inductance) && p->inductance>0 && isfinite(p->backEmfConstant) &&
      p->backEmfConstant>=0 && isfinite(p->electricalOffset);
}
void SimmyPmInit(SimmyPmState *s) { memset(s,0,sizeof(*s)); }
int SimmyPmStep(SimmyPmState *s, const SimmyPmParams *p, const double v[2],
                 double angle, double speed, double dt, SimmyPmOutput *output) {
  if (!s || !v || !output || !SimmyPmParamsValid(p) || !isfinite(angle) ||
      !isfinite(speed) || !isfinite(dt) || dt<=0) return 0;
  double theta=remainder(p->polePairs*(angle+.5*speed*dt)+p->electricalOffset,2*PI);
  double k[2]={-p->backEmfConstant*sin(theta),p->backEmfConstant*cos(theta)}, next[2];
  if (p->waveform==SIMMY_PM_TRAPEZOID) {
    double abc[3];
    for (int j=0;j<3;j++) abc[j]=-p->backEmfConstant*TrapSin(theta-2*PI*j/3);
    k[0]=(2*abc[0]-abc[1]-abc[2])/3;
    k[1]=(abc[1]-abc[2])/sqrt(3);
  }
  SimmyPmOutput o={0};
  for (int j=0;j<2;j++) {
    if (!isfinite(v[j]) || !isfinite(s->current[j])) return 0;
    next[j]=(p->inductance/dt*s->current[j]+v[j]-k[j]*speed)/(p->inductance/dt+p->resistance);
    if (!isfinite(next[j])) return 0;
    o.torque+=1.5*k[j]*next[j];
    o.magneticEnergy+=.75*p->inductance*next[j]*next[j];
    o.windingLossW+=1.5*p->resistance*next[j]*next[j];
    o.inputPower+=1.5*v[j]*next[j];
    double change=next[j]-s->current[j];
    o.numericalLossJ+=.75*p->inductance*change*change;
  }
  if (!isfinite(o.torque) || !isfinite(o.magneticEnergy) || !isfinite(o.windingLossW) ||
      !isfinite(o.inputPower) || !isfinite(o.numericalLossJ)) return 0;
  SimmyPmState result=*s;
  result.current[0]=next[0]; result.current[1]=next[1];
  result.windingHeatJ+=dt*o.windingLossW;
  result.inputJ+=dt*o.inputPower;
  result.mechanicalWorkJ+=dt*o.torque*speed;
  result.numericalLossJ+=o.numericalLossJ;
  if (!isfinite(result.windingHeatJ) || !isfinite(result.inputJ) ||
      !isfinite(result.mechanicalWorkJ) || !isfinite(result.numericalLossJ)) return 0;
  *s=result;
  *output=o;
  return 1;
}
