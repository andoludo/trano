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

    model StarNode "Massless node joining n conductances, solved in closed form (no algebraic loop)"
      parameter Integer n "Number of conductances";
      parameter Real G[n](each unit="W/K") "Conductances to the ports";
      Interfaces.HeatPort port[n];
      Interfaces.HeatPort node "Massless node: only heat flow sources may be connected to it";
      Interfaces.HeatPort rad
        "Radiant gains of emitters: at the conductance-weighted mean port temperature, spread as the node's";
    equation
      node.T = (sum(G[i]*port[i].T for i in 1:n) + node.Q_flow + rad.Q_flow)/sum(G);
      rad.T = sum(G[i]*port[i].T for i in 1:n)/sum(G);
      for i in 1:n loop
        port[i].Q_flow = G[i]*(port[i].T - node.T);
      end for;
    end StarNode;
  end Thermal;

  package Envelope "Surface heat transfer of the building envelope (after Buildings.ThermalZones.Detailed)"

    model InteriorConvection
      "Natural convection q = c*|dT|^(4/3) of Buildings.HeatTransfer.Convection.Interior, smooth at dT = 0"
      parameter Real A(unit="m2") "Surface area";
      parameter Real cPos "Coefficient when the solid is warmer than the fluid (1.3 wall, 1.51 floor, 0.76 ceiling)";
      parameter Real cNeg "Coefficient when the solid is colder than the fluid (1.3 wall, 0.76 floor, 1.51 ceiling)";
      parameter Real dT0(unit="K") = 0.1 "Temperature difference below which the law is regularised";
      Interfaces.HeatPort solid "Surface";
      Interfaces.HeatPort fluid "Air";
    equation
      solid.Q_flow = A*((cPos + cNeg)/2 + (cPos - cNeg)/2*(solid.T - fluid.T)/sqrt((solid.T - fluid.T)^2 + dT0^2))
        *(solid.T - fluid.T)*((solid.T - fluid.T)^2 + dT0^2)^(1/6);
      solid.Q_flow + fluid.Q_flow = 0;
    end InteriorConvection;

    model ExteriorConvection
      "Natural plus wind-driven convection of Buildings.HeatTransfer.Convection.Exterior (TemperatureWind)"
      parameter Real A(unit="m2") "Surface area";
      parameter Real cPos "Natural convection coefficient when the solid is warmer than the air";
      parameter Real cNeg "Natural convection coefficient when the solid is colder than the air";
      parameter Real azi(unit="rad") "Surface azimuth (0 south, pi/2 west)";
      parameter Real R = 1.67 "Surface roughness factor (1.67 for rough)";
      parameter Real dT0(unit="K") = 0.1 "Temperature difference below which the natural law is regularised";
      input Real v(unit="m/s") "Wind speed";
      input Real dir(unit="rad") "Wind direction (0 from the north)";
      Interfaces.HeatPort solid "Surface";
      Interfaces.HeatPort fluid "Outdoor air";
    protected
      Real W "Wind direction modifier: 1 windward, 0.5 leeward, 0.75 without wind";
      Real hF(unit="W/(m2.K)") "Forced convection coefficient";
    equation
      // windward share (1 when the wind blows on the surface, 0 when it is more than 100 degrees away),
      // blended to 0.75 at low wind speed as Buildings does
      W = 0.75 + (0.5*(0.5 + 0.5*(cos(azi + 3.141592653589793 - dir) - cos(1.7453292519943295))
        /sqrt((cos(azi + 3.141592653589793 - dir) - cos(1.7453292519943295))^2 + 0.0025)) - 0.25)
        *(0.5 + 0.5*(v - 0.25)/sqrt((v - 0.25)^2 + 0.015625));
      hF = 2.537*W*R*2/A^0.25*(v^2 + 0.0625)^0.25;
      solid.Q_flow = A*((cPos + cNeg)/2 + (cPos - cNeg)/2*(solid.T - fluid.T)/sqrt((solid.T - fluid.T)^2 + dT0^2))
        *(solid.T - fluid.T)*((solid.T - fluid.T)^2 + dT0^2)^(1/6) + A*hF*(solid.T - fluid.T);
      solid.Q_flow + fluid.Q_flow = 0;
    end ExteriorConvection;

    model SkyRadiation
      "Long-wave exchange of an exterior surface with sky and ground (Buildings SkyRadiationExchange, linear in T)"
      parameter Real k(unit="W/K4") "4*sigma*emissivity*area";
      parameter Real vieFacSky "View factor to the sky, (pi - tilt)/pi";
      input Real TOut(unit="K") "Outdoor air temperature";
      input Real TBlaSky(unit="K") "Black-body sky temperature";
      Interfaces.HeatPort port "Exterior surface";
    protected
      Real TEnv(unit="K") "Radiative temperature of the environment";
    equation
      TEnv = (vieFacSky*TBlaSky^4 + (1 - vieFacSky)*TOut^4)^0.25;
      port.Q_flow = k*TEnv^3*(port.T - TEnv);
    end SkyRadiation;

    model WindowSolar
      "Solar radiation transmitted and absorbed by a glazing, polynomials in the cosine of incidence (Buildings optics)"
      parameter Integer nPan "Number of panes";
      parameter Real AGla(unit="m2") "Glass area";
      parameter Real tau0 = 0 "Direct transmittance: tau0 + tau1*c + tau2*c^2 + tau3*c^3 + tau4*c^4, c = cos(incidence)";
      parameter Real tau1 = 0;
      parameter Real tau2 = 0;
      parameter Real tau3 = 0;
      parameter Real tau4 = 0;
      parameter Real tauDif "Hemispherical (diffuse) transmittance";
      parameter Real a0[nPan] "Direct absorptance of each pane: a0 + a1*c + ... + a4*c^4, outside first";
      parameter Real a1[nPan];
      parameter Real a2[nPan];
      parameter Real a3[nPan];
      parameter Real a4[nPan];
      parameter Real aDif[nPan] "Hemispherical (diffuse) absorptance of each pane";
      input Real HDir(unit="W/m2") "Direct irradiance on the window";
      input Real HDif(unit="W/m2") "Diffuse irradiance on the window";
      input Real cosInc "Cosine of the incidence angle of the beam radiation";
      output Real QTraDir(unit="W") "Transmitted direct solar radiation";
      output Real QTraDif(unit="W") "Transmitted diffuse solar radiation";
      Interfaces.HeatPort pane[nPan] "Panes: absorbed solar radiation";
    protected
      Real c "Cosine of incidence, 0 when the sun is behind the window";
    equation
      c = 0.5*(cosInc + sqrt(cosInc^2 + 1e-4));
      QTraDir = AGla*HDir*(tau0 + tau1*c + tau2*c^2 + tau3*c^3 + tau4*c^4);
      QTraDif = AGla*HDif*tauDif;
      for i in 1:nPan loop
        pane[i].Q_flow = -AGla*(HDir*(a0[i] + a1[i]*c + a2[i]*c^2 + a3[i]*c^3 + a4[i]*c^4) + HDif*aDif[i]);
      end for;
    end WindowSolar;

    model WindowExteriorConvection
      "Convection at the outer glass surface, A*(4 + 4*|v|) (Buildings.HeatTransfer.Windows ExteriorConvectionCoefficient)"
      parameter Real A(unit="m2") "Glass area";
      input Real v(unit="m/s") "Wind speed";
      Interfaces.HeatPort solid "Outer glass surface";
      Interfaces.HeatPort fluid "Outdoor air";
    equation
      solid.Q_flow = A*(4 + 4*sqrt(v^2 + 0.01))*(solid.T - fluid.T);
      solid.Q_flow + fluid.Q_flow = 0;
    end WindowExteriorConvection;

    model SolarDistribution
      "Transmitted solar radiation absorbed by the room surfaces (Buildings SolarRadiationExchange factors)"
      parameter Integer n "Number of surfaces";
      parameter Real kDir[n] "Share of the direct radiation absorbed by each surface (floors first)";
      parameter Real kDif[n] "Share of the diffuse radiation absorbed by each surface";
      input Real QDir(unit="W") "Transmitted direct radiation";
      input Real QDif(unit="W") "Transmitted diffuse radiation";
      Interfaces.HeatPort surface[n];
    equation
      for i in 1:n loop
        surface[i].Q_flow = -(kDir[i]*QDir + kDif[i]*QDif);
      end for;
    end SolarDistribution;
  end Envelope;

  package Hydronics
    "Water circuits after Buildings.Fluid: fixed flow directions, branch flows set by their valves, smooth mixing"
    constant Real cp(unit="J/(kg.K)") = 4184 "Specific heat capacity of water (Buildings.Media.Water)";

    model Radiator "EN 442 radiator (Buildings RadiatorEN442_2): nEle water and metal elements in series"
      parameter Integer nEle = 1 "Number of elements";
      parameter Real UAEle(unit="W/K") "Conductance of each element, from the nominal conditions";
      parameter Real n = 1.24 "Exponent of the EN 442 power law";
      parameter Real fraRad = 0.35 "Radiant share of the heat output";
      parameter Real C(unit="J/K") "Heat capacity of the water and of the dry mass";
      parameter Real T0(unit="K") = 293.15 "Initial water temperature";
      parameter Real dT0(unit="K") = 0.05 "Regularisation of the power law around a zero temperature difference";
      Real T[nEle](each unit="K", each start=T0, each fixed=true) "Water temperature of each element";
      Interfaces.WaterPort inlet;
      Interfaces.WaterPort outlet;
      Interfaces.HeatPort heatPortCon "Convective heat to the room air";
      Interfaces.HeatPort heatPortRad "Radiative heat to the room surfaces";
    protected
      Real QCon[nEle](each unit="W") "Convective heat output of each element";
      Real QRad[nEle](each unit="W") "Radiative heat output of each element";
    equation
      for i in 1:nEle loop
        QCon[i] = (1 - fraRad)*UAEle*(T[i] - heatPortCon.T)*((T[i] - heatPortCon.T)^2 + dT0^2)^((n - 1)/2);
        QRad[i] = fraRad*UAEle*(T[i] - heatPortRad.T)*((T[i] - heatPortRad.T)^2 + dT0^2)^((n - 1)/2);
      end for;
      C/nEle*der(T[1]) = inlet.m_flow*cp*(inlet.T - T[1]) - QCon[1] - QRad[1];
      for i in 2:nEle loop
        C/nEle*der(T[i]) = inlet.m_flow*cp*(T[i - 1] - T[i]) - QCon[i] - QRad[i];
      end for;
      inlet.m_flow + outlet.m_flow = 0;
      outlet.T = T[nEle];
      heatPortCon.Q_flow = -sum(QCon);
      heatPortRad.Q_flow = -sum(QRad);
    end Radiator;

    model Valve "Two-way equal-percentage valve setting the flow of its branch (installed characteristic)"
      parameter Real m_flow_nominal(unit="kg/s") "Flow at full opening";
      parameter Real R = 50 "Rangeability";
      parameter Real l = 1e-4 "Leakage at zero opening";
      parameter Real delta = 0.01 "Opening below which the characteristic turns linear to the leakage";
      parameter Real a = 1 "Valve authority: valve over branch pressure drop at nominal flow";
      parameter Real p = 2 "Pressure drop exponent of the branch: 2 turbulent, 1 linearized";
      input Real y "Opening, 0 to 1";
      Interfaces.WaterPort inlet;
      Interfaces.WaterPort outlet;
    protected
      Real phi "Flow coefficient of the valve relative to its nominal one";
    equation
      phi = l + (R^(y - 1) - l)*tanh(y/delta);
      inlet.m_flow = m_flow_nominal*phi/(a + (1 - a)*phi^p)^(1/p);
      inlet.m_flow + outlet.m_flow = 0;
      outlet.T = inlet.T;
    end Valve;

    model FlowValve "Valve setting the flow of its branch: the flow is the control (its opening follows locally)"
      input Real m_flow(unit="kg/s") "Mass flow rate of the branch";
      Interfaces.WaterPort inlet;
      Interfaces.WaterPort outlet;
    equation
      inlet.m_flow = m_flow;
      inlet.m_flow + outlet.m_flow = 0;
      outlet.T = inlet.T;
    end FlowValve;

    model ThreeWayValve "Mixing valve: the share y of the outlet flow comes from port_1 (supply), 1 - y from port_3"
      input Real y "Share of the supply, 0 to 1";
      Interfaces.WaterPort port_1 "Supply inlet";
      Interfaces.WaterPort port_3 "Return (bypass) inlet";
      Interfaces.WaterPort port_2 "Mixed outlet";
    equation
      port_1.m_flow + port_2.m_flow + port_3.m_flow = 0;
      port_1.m_flow = -y*port_2.m_flow;
      port_2.T = y*port_1.T + (1 - y)*port_3.T;
    end ThreeWayValve;

    model DivertingValve "Diverting valve: sends (1 - y)*m_flow_nominal from port_1 to the bypass port_3"
      parameter Real m_flow_nominal(unit="kg/s") "Bypass flow when the valve is fully on the bypass";
      input Real y "Position: 1 all to port_2 (the emitters), 0 the nominal flow to the bypass";
      Interfaces.WaterPort port_1 "Supply inlet";
      Interfaces.WaterPort port_2 "Outlet to the emitters";
      Interfaces.WaterPort port_3 "Bypass outlet";
    equation
      port_1.m_flow + port_2.m_flow + port_3.m_flow = 0;
      port_3.m_flow = -(1 - y)*m_flow_nominal;
      port_2.T = port_1.T;
      port_3.T = port_1.T;
    end DivertingValve;

    model Splitter "One inlet to n outlets at the inlet temperature"
      parameter Integer n = 2 "Number of outlets";
      Interfaces.WaterPort inlet;
      Interfaces.WaterPort outlet[n];
    equation
      inlet.m_flow + sum(outlet[i].m_flow for i in 1:n) = 0;
      for i in 1:n loop
        outlet[i].T = inlet.T;
      end for;
    end Splitter;

    model Mixer "n inlets to one outlet, ideal mixing (regularised at zero flow)"
      parameter Integer n = 2 "Number of inlets";
      parameter Real m_flow_small(unit="kg/s") = 1e-6 "Flow below which the outlet takes the mean inlet temperature";
      Interfaces.WaterPort inlet[n];
      Interfaces.WaterPort outlet;
    equation
      sum(inlet[i].m_flow for i in 1:n) + outlet.m_flow = 0;
      outlet.T = (sum(inlet[i].m_flow*inlet[i].T for i in 1:n) + m_flow_small*sum(inlet[i].T for i in 1:n)/n)
        /(sum(inlet[i].m_flow for i in 1:n) + m_flow_small);
    end Mixer;

    model Pump "Pass-through pump; electrical power at constant pressure head"
      parameter Real dp_nominal(unit="Pa") = 3e4 "Pressure head";
      parameter Real eta = 0.5 "Overall efficiency";
      parameter Real rho(unit="kg/m3") = 995.586 "Density of water";
      Interfaces.WaterPort inlet;
      Interfaces.WaterPort outlet;
      Real P(unit="W") "Electrical power";
    equation
      inlet.m_flow + outlet.m_flow = 0;
      outlet.T = inlet.T;
      P = dp_nominal*inlet.m_flow/rho/eta;
    end Pump;

    model TemperatureSensor "Pass-through: T is the water temperature"
      Interfaces.WaterPort inlet;
      Interfaces.WaterPort outlet;
      Real T(unit="K") "Measured temperature";
    equation
      inlet.m_flow + outlet.m_flow = 0;
      outlet.T = inlet.T;
      T = inlet.T;
    end TemperatureSensor;

    model PrescribedSupply "Ideal production: water leaves at the prescribed temperature; Q is the heat it takes"
      input Real TSup(unit="K") "Supply temperature";
      Interfaces.WaterPort inlet;
      Interfaces.WaterPort outlet;
      Real Q(unit="W") "Heat delivered to the water";
    equation
      inlet.m_flow + outlet.m_flow = 0;
      outlet.T = TSup;
      Q = inlet.m_flow*cp*(TSup - inlet.T);
    end PrescribedSupply;

    model Boiler "Boiler (Buildings BoilerPolynomial): water and metal heated by the part load ratio y"
      parameter Real Q_flow_nominal(unit="W") "Nominal heating power";
      parameter Real a1 = 0.9 "Efficiency: a1 + a2*y + a3*y^2 + (a4 + a5*y + a6*y^2)*T (QuadraticLinear curve)";
      parameter Real a2 = 0;
      parameter Real a3 = 0;
      parameter Real a4 = 0;
      parameter Real a5 = 0;
      parameter Real a6 = 0;
      parameter Real eta_nominal = 0.9 "Efficiency at nominal conditions";
      parameter Real UA(unit="W/K") = 0 "Heat loss conductance to the boiler room";
      parameter Real TAmb(unit="K") = 288.15 "Temperature of the boiler room";
      parameter Real C(unit="J/K") "Heat capacity of the water and of the dry mass";
      parameter Real T0(unit="K") = 293.15 "Initial water temperature";
      parameter Real E0(unit="kWh") = 0 "Initial reading of the fuel meter";
      input Real y "Part load ratio, 0 to 1 (control)";
      Real T(unit="K", start=T0, fixed=true) "Water temperature";
      Real EFue(unit="kWh", start=E0, fixed=true) "Fuel meter";
      Interfaces.WaterPort inlet;
      Interfaces.WaterPort outlet;
      Real QFue(unit="W") "Fuel power";
      Real QWat(unit="W") "Heat to the water";
    protected
      Real eta "Efficiency";
    equation
      eta = a1 + a2*y + a3*y^2 + (a4 + a5*y + a6*y^2)*T;
      QFue = y*Q_flow_nominal/eta_nominal;
      QWat = eta*QFue + UA*(TAmb - T);
      C*der(T) = inlet.m_flow*cp*(inlet.T - T) + QWat;
      der(EFue) = QFue/3.6e6;
      inlet.m_flow + outlet.m_flow = 0;
      outlet.T = T;
    end Boiler;

    model HeatPump
      "Air-to-water heat pump (Buildings ModularReversible TableData2D) charging a mixed buffer, compressor speed y"
      parameter Integer m "Number of interior knots of the source temperature";
      parameter Real tEva0 "First knot of the source temperature [degC]";
      parameter Real tEva[m] "Interior knots of the source temperature [degC]";
      parameter Real dTEva = 1 "Smoothing of the kinks of the table interpolation [K]";
      parameter Real aQ0 "Heating capacity per unit speed: (q0 + q1*TCon + q2*TCon^2), TCon in degC; each qk is";
      parameter Real bQ0 "  ak + bk*(TEva - tEva0) + sum(ck[i]*hinge(TEva - tEva[i])), linear between the knots";
      parameter Real cQ0[m];
      parameter Real aQ1;
      parameter Real bQ1;
      parameter Real cQ1[m];
      parameter Real aQ2;
      parameter Real bQ2;
      parameter Real cQ2[m];
      parameter Real aP0 "Electrical power per unit speed, as the heating capacity";
      parameter Real bP0;
      parameter Real cP0[m];
      parameter Real aP1;
      parameter Real bP1;
      parameter Real cP1[m];
      parameter Real aP2;
      parameter Real bP2;
      parameter Real cP2[m];
      parameter Real scaFac "Scaling of the table data to the nominal capacity";
      parameter Real dTCon(unit="K") "Condenser outlet above the buffer at full speed";
      parameter Real UA(unit="W/K") = 0 "Heat loss conductance of the buffer";
      parameter Real TAmb(unit="K") = 288.15 "Temperature around the buffer";
      parameter Real C(unit="J/K") "Heat capacity of the buffer and condenser water";
      parameter Real T0(unit="K") = 318.15 "Initial buffer temperature";
      parameter Real E0(unit="kWh") = 0 "Initial reading of the electricity meter";
      input Real y "Compressor speed, 0 to 1 (control)";
      input Real TEva(unit="K") "Source (outdoor air) temperature";
      Real T(unit="K", start=T0, fixed=true) "Buffer temperature";
      Real EEle(unit="kWh", start=E0, fixed=true) "Electricity meter";
      Interfaces.WaterPort inlet;
      Interfaces.WaterPort outlet;
      Real QCon(unit="W") "Heat to the water";
      Real PEle(unit="W") "Electrical power of the compressor";
    protected
      Real te "Source temperature [degC]";
      Real tc "Condenser outlet temperature [degC]";
      Real q[3] "Coefficients of the heating capacity in the condenser temperature";
      Real p[3] "Coefficients of the electrical power in the condenser temperature";
    equation
      te = TEva - 273.15;
      tc = T - 273.15 + dTCon*y;
      q[1] = aQ0 + bQ0*(te - tEva0) + sum(cQ0[i]*0.5*(te - tEva[i] + sqrt((te - tEva[i])^2 + dTEva^2)) for i in 1:m);
      q[2] = aQ1 + bQ1*(te - tEva0) + sum(cQ1[i]*0.5*(te - tEva[i] + sqrt((te - tEva[i])^2 + dTEva^2)) for i in 1:m);
      q[3] = aQ2 + bQ2*(te - tEva0) + sum(cQ2[i]*0.5*(te - tEva[i] + sqrt((te - tEva[i])^2 + dTEva^2)) for i in 1:m);
      p[1] = aP0 + bP0*(te - tEva0) + sum(cP0[i]*0.5*(te - tEva[i] + sqrt((te - tEva[i])^2 + dTEva^2)) for i in 1:m);
      p[2] = aP1 + bP1*(te - tEva0) + sum(cP1[i]*0.5*(te - tEva[i] + sqrt((te - tEva[i])^2 + dTEva^2)) for i in 1:m);
      p[3] = aP2 + bP2*(te - tEva0) + sum(cP2[i]*0.5*(te - tEva[i] + sqrt((te - tEva[i])^2 + dTEva^2)) for i in 1:m);
      QCon = y*scaFac*(q[1] + q[2]*tc + q[3]*tc^2);
      PEle = y*scaFac*(p[1] + p[2]*tc + p[3]*tc^2);
      C*der(T) = inlet.m_flow*cp*(inlet.T - T) + QCon + UA*(TAmb - T);
      der(EEle) = PEle/3.6e6;
      inlet.m_flow + outlet.m_flow = 0;
      outlet.T = T;
    end HeatPump;

    model Reference "One per closed circuit (the expansion vessel): drops the mass balance the loop makes redundant"
      Interfaces.WaterPort inlet;
      Interfaces.WaterPort outlet;
    equation
      outlet.T = inlet.T;
    end Reference;
  end Hydronics;

  package Ventilation "Mechanical ventilation (after trano's Buildings system D)"

    model HeatRecoveryUnit
      "Balanced ventilation of n zones through a constant-effectiveness heat recovery (Buildings ConstantEffectiveness)"
      parameter Integer n "Number of ventilated zones";
      parameter Real mNom[n](each unit="kg/s") "Supply (and extract) air flow of each zone at full speed";
      parameter Real eps = 0.8 "Effectiveness of the heat recovery";
      parameter Real cpAir(unit="J/(kg.K)") = 1006 "Specific heat capacity of air";
      parameter Real dTFan(unit="K") = 0 "Heating of the supply air by the fans (constant pressure fans)";
      parameter Real PNom(unit="W") = 0 "Electrical power of the fans at full speed";
      parameter Real E0(unit="kWh") = 0 "Initial reading of the electricity meter";
      input Real y "Air flow relative to the nominal one (control), 0 to 1";
      input Real TOut(unit="K") "Outdoor air temperature";
      Real EFan(unit="kWh", start=E0, fixed=true) "Electricity meter of the fans";
      Interfaces.HeatPort zone[n] "Air of the ventilated zones";
      Real TSup(unit="K") "Supply air temperature";
      Real PFan(unit="W") "Electrical power of the fans";
    protected
      Real TExt(unit="K") "Extract air temperature: the zone air mixed by flow";
    equation
      TExt = sum(mNom[i]*zone[i].T for i in 1:n)/sum(mNom[i] for i in 1:n);
      TSup = TOut + eps*(TExt - TOut) + dTFan;
      for i in 1:n loop
        zone[i].Q_flow = y*mNom[i]*cpAir*(zone[i].T - TSup);
      end for;
      PFan = PNom*y;
      der(EFan) = PFan/3.6e6;
    end HeatRecoveryUnit;
  end Ventilation;

  package Electrical "Electricity production"

    model Photovoltaic "PV array (Buildings PVSimpleOriented): P = A*fAct*eta*eta_DCAC*(HDir + HDif)"
      parameter Real A(unit="m2") "Module area";
      parameter Real eta = 0.2 "Module efficiency";
      parameter Real fAct = 0.9 "Active share of the area";
      parameter Real eta_DCAC = 0.9 "Inverter efficiency";
      parameter Real E0(unit="kWh") = 0 "Initial reading of the production meter";
      input Real HDir(unit="W/m2") "Direct irradiance on the array";
      input Real HDif(unit="W/m2") "Diffuse irradiance on the array";
      Real EPro(unit="kWh", start=E0, fixed=true) "Electricity produced";
      Real P(unit="W") "Electrical power produced";
    equation
      P = A*fAct*eta*eta_DCAC*(HDir + HDif);
      der(EPro) = P/3.6e6;
    end Photovoltaic;
  end Electrical;

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
