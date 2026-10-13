package Components "Primitive components of Trano.MPC: connectable models that translate into an explicit CasADi ODE"

  package Interfaces "Connectors"

    connector HeatPort "Thermal port: temperature and heat flow rate into the component"
      Real T(unit="K") "Port temperature";
      flow Real Q_flow(unit="W") "Heat flow rate into the component";
    end HeatPort;

    connector WaterPort "Water port: mass flow rate into the component in the design direction and its temperature"
      flow Real m_flow(unit="kg/s") "Mass flow rate into the component";
      Real T(unit="K") "Temperature of the water crossing the port";
    end WaterPort;
  end Interfaces;

  package Thermal "Lumped thermal elements"

    model HeatCapacitor "Lumped heat capacity: one state"
      parameter Real C(unit="J/K") "Heat capacity";
      parameter Real T0(unit="K") = 294.15 "Initial temperature";
      Real T(unit="K", start=T0, fixed=true) "Temperature";
      Interfaces.HeatPort port;
    equation
      port.T = T;
      C*der(T) = port.Q_flow;
    end HeatCapacitor;

    model ThermalResistor "Thermal resistance between two ports"
      parameter Real R(unit="K/W") "Thermal resistance";
      Interfaces.HeatPort port_a;
      Interfaces.HeatPort port_b;
    equation
      port_a.Q_flow = (port_a.T - port_b.T)/R;
      port_a.Q_flow + port_b.Q_flow = 0;
    end ThermalResistor;

    model ThermalConductor "Thermal conductance between two ports"
      parameter Real G(unit="W/K") "Thermal conductance";
      Interfaces.HeatPort port_a;
      Interfaces.HeatPort port_b;
    equation
      port_a.Q_flow = G*(port_a.T - port_b.T);
      port_a.Q_flow + port_b.Q_flow = 0;
    end ThermalConductor;

    model StarNode3 "Massless node joining three conductances, solved in closed form (no algebraic loop)"
      parameter Real G1(unit="W/K") "Conductance to port_1";
      parameter Real G2(unit="W/K") "Conductance to port_2";
      parameter Real G3(unit="W/K") "Conductance to port_3";
      Interfaces.HeatPort port_1;
      Interfaces.HeatPort port_2;
      Interfaces.HeatPort port_3;
      Interfaces.HeatPort node "Massless node: only heat flow sources may be connected to it";
    equation
      node.T = (G1*port_1.T + G2*port_2.T + G3*port_3.T + node.Q_flow)/(G1 + G2 + G3);
      port_1.Q_flow = G1*(port_1.T - node.T);
      port_2.Q_flow = G2*(port_2.T - node.T);
      port_3.Q_flow = G3*(port_3.T - node.T);
    end StarNode3;
  end Thermal;

  package Sources "Boundary conditions and heat inputs"

    model PrescribedTemperature "Temperature imposed by an input"
      input Real T(unit="K") "Temperature";
      Interfaces.HeatPort port;
    equation
      port.T = T;
    end PrescribedTemperature;

    model FixedTemperature "Constant temperature"
      parameter Real T(unit="K") "Temperature";
      Interfaces.HeatPort port;
    equation
      port.T = T;
    end FixedTemperature;

    model PrescribedHeatFlow "Heat flow rate injected into the port by an input"
      input Real Q_flow(unit="W") "Injected heat flow rate";
      Interfaces.HeatPort port;
    equation
      port.Q_flow = -Q_flow;
    end PrescribedHeatFlow;

    model ScaledHeatFlow "Heat flow rate k*u injected into the port (solar gains, split internal gains)"
      parameter Real k "Scaling factor (e.g. an effective area in m2 for an irradiance in W/m2)";
      input Real u "Input signal";
      Interfaces.HeatPort port;
    equation
      port.Q_flow = -k*u;
    end ScaledHeatFlow;
  end Sources;

  annotation (Documentation(info="<html>
<p>Primitive components of the <code>Trano.MPC</code> library. trano assembles them, with <code>connect()</code>
statements, into the zones, emitters and hydronic networks of a building, and the resulting model translates
into an explicit CasADi ODE (rumoca &ge; 0.10.3) for simulation, estimation and model predictive control with IPOPT.</p>
<p>Rules kept by every component: own connectors (no short-class connectors), every node joined by
<code>connect()</code> has a heat capacity or is solved in closed form inside its component, smooth equations
(no events, tables or functions), parameters bound to literal values.</p>
</html>"));
end Components;
