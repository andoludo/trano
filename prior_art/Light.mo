// Prototype of the connectable, optimizer-ready component library evaluated in PRIOR_ART.md (section 1b).
// Light.House is the three-zone benchmark house of tests/models/three_zones_mpc.yaml, rebuilt from components.
package Light "Lightweight connectable thermal components (no MSL dependency)"
  connector HeatPort
    Real T(unit = "K");
    flow Real Q_flow(unit = "W");
  end HeatPort;
  model Capacitor
    parameter Real C(unit = "J/K");
    parameter Real T0(unit = "K") = 294.15;
    Real T(unit = "K", start = T0, fixed = true);
    HeatPort port;
  equation
    port.T = T;
    C*der(T) = port.Q_flow;
  end Capacitor;
  model Resistor
    parameter Real R(unit = "K/W");
    HeatPort a;
    HeatPort b;
  equation
    a.Q_flow = (a.T - b.T)/R;
    a.Q_flow + b.Q_flow = 0;
  end Resistor;
  model Conductor
    parameter Real G(unit = "W/K");
    HeatPort a;
    HeatPort b;
  equation
    a.Q_flow = G*(a.T - b.T);
    a.Q_flow + b.Q_flow = 0;
  end Conductor;
  model PrescribedTemperature
    input Real T(unit = "K");
    HeatPort port;
  equation
    port.T = T;
  end PrescribedTemperature;
  model FixedTemperature
    parameter Real T(unit = "K");
    HeatPort port;
  equation
    port.T = T;
  end FixedTemperature;
  model HeatFlow "Heat injected into the port (gains, heating)"
    input Real Q(unit = "W");
    HeatPort port;
  equation
    port.Q_flow = -Q;
  end HeatFlow;
  model Solar "Irradiance times an effective area"
    parameter Real A(unit = "m2");
    input Real H(unit = "W/m2");
    HeatPort port;
  equation
    port.Q_flow = -A*H;
  end Solar;
  model ZoneR3C2 "Indoor air + envelope, ports to outdoor and ground"
    parameter Real Ci, Ce, Rie, Ria, Rea, Reg;
    Capacitor air(C = Ci);
    Capacitor env(C = Ce);
    Resistor rie(R = Rie);
    Resistor ria(R = Ria);
    Resistor rea(R = Rea);
    Resistor reg(R = Reg);
    HeatPort outdoor;
    HeatPort ground;
    HeatPort indoor "indoor air node";
    HeatPort envelope "envelope node";
  equation
    connect(air.port, indoor);
    connect(env.port, envelope);
    connect(air.port, rie.a);
    connect(rie.b, env.port);
    connect(air.port, ria.a);
    connect(ria.b, outdoor);
    connect(env.port, rea.a);
    connect(rea.b, outdoor);
    connect(env.port, reg.a);
    connect(reg.b, ground);
  end ZoneR3C2;
  model House
    input Real TOut;
    input Real HSol_azi0_til90;
    input Real HSol_azi90_til90;
    input Real HSol_azi180_til90;
    input Real space_001_QInt;
    input Real space_001_QHea;
    input Real space_002_QInt;
    input Real space_002_QHea;
    input Real space_003_QInt;
    input Real space_003_QHea;
    Light.PrescribedTemperature outdoor;
    Light.FixedTemperature ground(T = 283.15);
    Light.Capacitor space_001_Ti(C = 1507500.0, T0 = 294.15);
    Light.Capacitor space_001_Te(C = 90700336.0, T0 = 294.15);
    Light.Resistor space_001_Rie(R = 0.00189727652);
    Light.Resistor space_001_Ria(R = 0.01908541473);
    Light.Resistor space_001_Rea(R = 0.0189955422);
    Light.Resistor space_001_Reg(R = 0.004910714286);
    Light.HeatFlow space_001_gains;
    Light.Solar sol_space_001_gA_azi0_til90(A = 2.41115);
    Light.Solar sol_space_001_aE_azi0_til90(A = 0.09936048148);
    Light.Solar sol_space_001_aE_azi90_til90(A = 0.198720963);
    Light.Solar sol_space_001_gA_azi180_til90(A = 0.96446);
    Light.Solar sol_space_001_aE_azi180_til90(A = 0.3179535407);
    Light.Capacitor space_002_Ti(C = 1055250.0, T0 = 294.15);
    Light.Capacitor space_002_Te(C = 59214963.01, T0 = 294.15);
    Light.Resistor space_002_Rie(R = 0.003373393931);
    Light.Resistor space_002_Ria(R = 0.02724734992);
    Light.Resistor space_002_Rea(R = 0.02410916602);
    Light.Resistor space_002_Reg(R = 0.009821428571);
    Light.HeatFlow space_002_gains;
    Light.Solar sol_space_002_gA_azi0_til90(A = 1.83015);
    Light.Solar sol_space_002_aE_azi0_til90(A = 0.1262223993);
    Light.Solar sol_space_002_aE_azi90_til90(A = 0.1577779991);
    Light.Solar sol_space_002_gA_azi180_til90(A = 0.73206);
    Light.Solar sol_space_002_aE_azi180_til90(A = 0.2019558388);
    Light.Capacitor space_003_Ti(C = 753750.0, T0 = 294.15);
    Light.Capacitor space_003_Te(C = 77987230.38, T0 = 294.15);
    Light.Resistor space_003_Rie(R = 0.003850706005);
    Light.Resistor space_003_Ria(R = 0.04776119403);
    Light.Resistor space_003_Rea(R = 0.05767619048);
    Light.Resistor space_003_Reg(R = 0.009821428571);
    Light.HeatFlow space_003_gains;
    Light.Solar sol_space_003_aE_azi180_til90(A = 0.2059159993);
    Light.Conductor H_space_001_space_002(G = 7.084711552);
    Light.Conductor H_space_002_space_003(G = 5.651442311);
  equation
    outdoor.T = TOut;
    connect(space_001_Ti.port, space_001_Rie.a);
    connect(space_001_Rie.b, space_001_Te.port);
    connect(space_001_Ti.port, space_001_Ria.a);
    connect(space_001_Ria.b, outdoor.port);
    connect(space_001_Te.port, space_001_Rea.a);
    connect(space_001_Rea.b, outdoor.port);
    connect(space_001_Te.port, space_001_Reg.a);
    connect(space_001_Reg.b, ground.port);
    space_001_gains.Q = space_001_QInt + space_001_QHea;
    connect(space_001_gains.port, space_001_Ti.port);
    sol_space_001_gA_azi0_til90.H = HSol_azi0_til90;
    connect(sol_space_001_gA_azi0_til90.port, space_001_Ti.port);
    sol_space_001_aE_azi0_til90.H = HSol_azi0_til90;
    connect(sol_space_001_aE_azi0_til90.port, space_001_Te.port);
    sol_space_001_aE_azi90_til90.H = HSol_azi90_til90;
    connect(sol_space_001_aE_azi90_til90.port, space_001_Te.port);
    sol_space_001_gA_azi180_til90.H = HSol_azi180_til90;
    connect(sol_space_001_gA_azi180_til90.port, space_001_Ti.port);
    sol_space_001_aE_azi180_til90.H = HSol_azi180_til90;
    connect(sol_space_001_aE_azi180_til90.port, space_001_Te.port);
    connect(space_002_Ti.port, space_002_Rie.a);
    connect(space_002_Rie.b, space_002_Te.port);
    connect(space_002_Ti.port, space_002_Ria.a);
    connect(space_002_Ria.b, outdoor.port);
    connect(space_002_Te.port, space_002_Rea.a);
    connect(space_002_Rea.b, outdoor.port);
    connect(space_002_Te.port, space_002_Reg.a);
    connect(space_002_Reg.b, ground.port);
    space_002_gains.Q = space_002_QInt + space_002_QHea;
    connect(space_002_gains.port, space_002_Ti.port);
    sol_space_002_gA_azi0_til90.H = HSol_azi0_til90;
    connect(sol_space_002_gA_azi0_til90.port, space_002_Ti.port);
    sol_space_002_aE_azi0_til90.H = HSol_azi0_til90;
    connect(sol_space_002_aE_azi0_til90.port, space_002_Te.port);
    sol_space_002_aE_azi90_til90.H = HSol_azi90_til90;
    connect(sol_space_002_aE_azi90_til90.port, space_002_Te.port);
    sol_space_002_gA_azi180_til90.H = HSol_azi180_til90;
    connect(sol_space_002_gA_azi180_til90.port, space_002_Ti.port);
    sol_space_002_aE_azi180_til90.H = HSol_azi180_til90;
    connect(sol_space_002_aE_azi180_til90.port, space_002_Te.port);
    connect(space_003_Ti.port, space_003_Rie.a);
    connect(space_003_Rie.b, space_003_Te.port);
    connect(space_003_Ti.port, space_003_Ria.a);
    connect(space_003_Ria.b, outdoor.port);
    connect(space_003_Te.port, space_003_Rea.a);
    connect(space_003_Rea.b, outdoor.port);
    connect(space_003_Te.port, space_003_Reg.a);
    connect(space_003_Reg.b, ground.port);
    space_003_gains.Q = space_003_QInt + space_003_QHea;
    connect(space_003_gains.port, space_003_Ti.port);
    sol_space_003_aE_azi180_til90.H = HSol_azi180_til90;
    connect(sol_space_003_aE_azi180_til90.port, space_003_Te.port);
    connect(space_001_Ti.port, H_space_001_space_002.a);
    connect(H_space_001_space_002.b, space_002_Ti.port);
    connect(space_002_Ti.port, H_space_002_space_003.a);
    connect(H_space_002_space_003.b, space_003_Ti.port);
  end House;
end Light;
