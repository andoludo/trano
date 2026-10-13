// Spike for MPC_LIBRARY_PLAN.md (stage 4): connectable hydronic circuit exported by rumoca main to a CasADi ODE.
// LightHydronics.Circuit: boiler, three-way mixing valve, pump, two radiator branches, return split, reference.
package LightHydronics "Spike: connectable, optimizer-ready hydronic components (fixed flow direction, flows set by valves)"
  constant Real cp = 4184 "Specific heat capacity of water";

  connector HeatPort
    Real T;
    flow Real Q_flow;
  end HeatPort;

  connector WaterPort "Water port: mass flow into the component (positive in design direction) and its temperature"
    flow Real m_flow;
    Real T;
  end WaterPort;

  model Capacitor
    parameter Real C;
    parameter Real T0 = 293.15;
    Real T(start = T0, fixed = true);
    HeatPort port;
  equation
    port.T = T;
    C*der(T) = port.Q_flow;
  end Capacitor;

  model Resistor
    parameter Real R;
    HeatPort a;
    HeatPort b;
  equation
    a.Q_flow = (a.T - b.T)/R;
    a.Q_flow + b.Q_flow = 0;
  end Resistor;

  model Outdoor
    input Real T;
    HeatPort port;
  equation
    port.T = T;
  end Outdoor;

  model Valve "Two-way valve with equal-percentage characteristic: sets the flow of its branch"
    parameter Real m_flow_nominal;
    parameter Real R = 50 "Rangeability";
    input Real y "Opening";
    WaterPort inlet;
    WaterPort outlet;
  equation
    inlet.m_flow = m_flow_nominal*exp(log(R)*(y - 1));
    inlet.m_flow + outlet.m_flow = 0;
    outlet.T = inlet.T;
  end Valve;

  model Pump "Pass-through pump; electrical power of a constant-dp pump"
    parameter Real dp_nominal = 3e4;
    parameter Real eta = 0.5;
    WaterPort inlet;
    WaterPort outlet;
    Real P "Electrical power";
  equation
    inlet.m_flow + outlet.m_flow = 0;
    outlet.T = inlet.T;
    P = dp_nominal*inlet.m_flow/1000/eta;
  end Pump;

  model Radiator "Lumped radiator: water and metal node, linear UA (spike)"
    parameter Real C;
    parameter Real UA;
    parameter Real T0 = 313.15;
    Real T(start = T0, fixed = true);
    WaterPort inlet;
    WaterPort outlet;
    HeatPort heatPort;
  equation
    inlet.m_flow + outlet.m_flow = 0;
    C*der(T) = inlet.m_flow*cp*(inlet.T - T) - UA*(T - heatPort.T);
    heatPort.Q_flow = -UA*(T - heatPort.T);
    outlet.T = T;
  end Radiator;

  model Boiler "Water volume heated by a controlled heat input"
    parameter Real C;
    parameter Real T0 = 343.15;
    input Real Q;
    Real T(start = T0, fixed = true);
    WaterPort inlet;
    WaterPort outlet;
  equation
    inlet.m_flow + outlet.m_flow = 0;
    C*der(T) = Q + inlet.m_flow*cp*(inlet.T - T);
    outlet.T = T;
  end Boiler;

  model Splitter
    WaterPort inlet;
    WaterPort outlet1;
    WaterPort outlet2;
  equation
    inlet.m_flow + outlet1.m_flow + outlet2.m_flow = 0;
    outlet1.T = inlet.T;
    outlet2.T = inlet.T;
  end Splitter;

  model Mixer
    WaterPort inlet1;
    WaterPort inlet2;
    WaterPort outlet;
  equation
    inlet1.m_flow + inlet2.m_flow + outlet.m_flow = 0;
    outlet.T = (inlet1.m_flow*inlet1.T + inlet2.m_flow*inlet2.T)/(inlet1.m_flow + inlet2.m_flow);
  end Mixer;

  model ThreeWayValve "Mixing valve: share y of the outlet flow comes from port_1 (supply), 1 - y from port_3 (return)"
    input Real y;
    WaterPort port_1;
    WaterPort port_3;
    WaterPort port_2;
  equation
    port_1.m_flow + port_2.m_flow + port_3.m_flow = 0;
    port_1.m_flow = -y*port_2.m_flow;
    port_2.T = y*port_1.T + (1 - y)*port_3.T;
  end ThreeWayValve;

  model Reference "One per closed hydraulic network: drops the redundant mass balance (expansion vessel)"
    WaterPort inlet;
    WaterPort outlet;
  equation
    outlet.T = inlet.T;
  end Reference;

  model Circuit "Boiler -> three-way mixing valve -> pump -> two radiator branches -> return split"
    input Real TOut;
    input Real QBoi "Boiler heat [W] (control)";
    input Real yMix "Three-way valve position (control)";
    input Real y1 "Radiator valve 1 (control)";
    input Real y2 "Radiator valve 2 (control)";
    Outdoor out(T = TOut);
    Capacitor zon1(C = 2e6);
    Capacitor zon2(C = 1.5e6);
    Resistor r1(R = 0.01);
    Resistor r2(R = 0.012);
    Boiler boi(C = 4184*20, Q = QBoi);
    ThreeWayValve mix(y = yMix);
    Pump pum;
    Splitter spl;
    Valve val1(m_flow_nominal = 0.08, y = y1);
    Valve val2(m_flow_nominal = 0.06, y = y2);
    Radiator rad1(C = 4184*15 + 30000, UA = 60);
    Radiator rad2(C = 4184*10 + 20000, UA = 45);
    Mixer ret;
    Splitter retSpl;
    Reference ref;
  equation
    connect(zon1.port, r1.a);
    connect(r1.b, out.port);
    connect(zon2.port, r2.a);
    connect(r2.b, out.port);
    connect(boi.outlet, mix.port_1);
    connect(mix.port_2, pum.inlet);
    connect(pum.outlet, spl.inlet);
    connect(spl.outlet1, val1.inlet);
    connect(spl.outlet2, val2.inlet);
    connect(val1.outlet, rad1.inlet);
    connect(val2.outlet, rad2.inlet);
    connect(rad1.heatPort, zon1.port);
    connect(rad2.heatPort, zon2.port);
    connect(rad1.outlet, ret.inlet1);
    connect(rad2.outlet, ret.inlet2);
    connect(ret.outlet, retSpl.inlet);
    connect(retSpl.outlet1, mix.port_3);
    connect(retSpl.outlet2, ref.inlet);
    connect(ref.outlet, boi.inlet);
  end Circuit;
end LightHydronics;
