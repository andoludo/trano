package FastModel "Building models for model predictive control: close to Buildings, translated into an explicit CasADi ODE"
  extends FastModel.Icons.Package;

  constant Real pi = 3.141592653589793;
  constant Real sigma(unit="W/(m2.K4)") = 5.670374419e-8 "Stefan-Boltzmann constant";
  constant Real cpWat(unit="J/(kg.K)") = 4184 "Specific heat capacity of water (Buildings.Media.Water)";
  constant Real rhoWat(unit="kg/m3") = 995.586 "Density of water (Buildings.Media.Water)";
  constant Real cpAir(unit="J/(kg.K)") = 1006 "Specific heat capacity of air";
  constant Real rhoAir(unit="kg/m3") = 1.2 "Density of air";

  package Icons "Icons"
    partial package Package "Package icon"
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(lineColor={200,200,200}, fillColor={248,248,248}, fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}}, radius=25),
          Rectangle(lineColor={128,128,128}, extent={{-100,-100},{100,100}}, radius=25)}));
    end Package;
  end Icons;

  package Types "Types and constants"
    package Tilt "Tilt of a surface, as Buildings.Types.Tilt [rad]"
      constant Real Ceiling = 0 "Ceiling or roof: the surface faces up";
      constant Real Wall = 1.5707963267948966 "Wall";
      constant Real Floor = 3.141592653589793 "Floor: the surface faces down";
    end Tilt;

    package EfficiencyCurves "Efficiency curves of boilers, as Buildings.Fluid.Types.EfficiencyCurves"
      constant Integer Constant = 1 "eta = a[1]";
      constant Integer Polynomial = 2 "eta = a[1] + a[2]*y + a[3]*y^2 + ...";
      constant Integer QuadraticLinear = 3 "eta = a[1] + a[2]*y + a[3]*y^2 + (a[4] + a[5]*y + a[6]*y^2)*T";
    end EfficiencyCurves;
  end Types;

  package Utilities "Smooth functions (after Buildings.Utilities.Math.Functions) and parameter computations"
    function regStep "Once continuously differentiable step from y2 (x < -x_small) to y1 (x > x_small)"
      input Real x;
      input Real y1;
      input Real y2;
      input Real x_small = 1e-5;
      output Real y;
    algorithm
      y := if x > x_small then y1 else if x < -x_small then y2 else (x/x_small)*((x/x_small)^2 - 3)*(y2 - y1)/4 + (y1 + y2)/2;
    end regStep;

    function smoothMax "Once continuously differentiable approximation of max(x1, x2)"
      input Real x1;
      input Real x2;
      input Real deltaX;
      output Real y;
    algorithm
      y := regStep(x1 - x2, x1, x2, deltaX);
    end smoothMax;

    function smoothMin "Once continuously differentiable approximation of min(x1, x2)"
      input Real x1;
      input Real x2;
      input Real deltaX;
      output Real y;
    algorithm
      y := regStep(x2 - x1, x1, x2, deltaX);
    end smoothMin;

    function smoothLimit "Once continuously differentiable limitation of x to [l, u]"
      input Real x;
      input Real l;
      input Real u;
      input Real deltaX;
      output Real y;
    algorithm
      y := smoothMin(smoothMax(x, l + deltaX, deltaX), u - deltaX, deltaX);
    end smoothLimit;

    function positivePart "Smooth max(x, 0): 0.5*(x + sqrt(x^2 + delta^2))"
      input Real x;
      input Real delta;
      output Real y;
    algorithm
      y := 0.5*(x + sqrt(x^2 + delta^2));
    end positivePart;

    function naturalConvection
      "Heat flow from a surface to the air q = A*c*|dT|^(4/3) (Buildings natural convection), smooth at dT = 0"
      input Real dT(unit="K") "Surface minus air temperature";
      input Real A(unit="m2") "Surface area";
      input Real cPos "Coefficient when the surface is warmer than the air";
      input Real cNeg "Coefficient when the surface is colder than the air";
      input Real dT0(unit="K") = 0.1 "Temperature difference below which the law is regularised";
      output Real Q_flow(unit="W");
    algorithm
      Q_flow := A*((cPos + cNeg)/2 + (cPos - cNeg)/2*dT/sqrt(dT^2 + dT0^2))*dT*(dT^2 + dT0^2)^(1/6);
    end naturalConvection;

    function convectionCoefficient
      "Natural convection coefficient of Buildings for a surface tilt: wall 1.3, ceiling 0.76/1.51, floor 1.51/0.76"
      input Real til(unit="rad") "Tilt: 0 for a ceiling, pi/2 for a wall, pi for a floor";
      input Boolean warmer "true: the coefficient when the surface is warmer than the air";
      output Real c;
    algorithm
      if abs(sin(til)) < 1e-6 and cos(til) > 0 then
        c := if warmer then 0.76 else 1.51;
      elseif abs(sin(til)) < 1e-6 and cos(til) < 0 then
        c := if warmer then 1.51 else 0.76;
      else
        c := 1.3;
      end if;
    end convectionCoefficient;

    function polynomialFit "Least-squares quartic c[1] + c[2]*x + ... + c[5]*x^4 through 10 points (x, y)"
      input Real x[10];
      input Real y[10];
      output Real c[5];
    protected
      Real A[5, 5];
      Real b[5];
      Real factor;
    algorithm
      // normal equations
      for i in 1:5 loop
        for j in 1:5 loop
          A[i, j] := sum(x[k]^(i + j - 2) for k in 1:10);
        end for;
        b[i] := sum(y[k]*x[k]^(i - 1) for k in 1:10);
      end for;
      // Gaussian elimination (the normal matrix is symmetric positive definite: no pivoting needed)
      for i in 1:5 loop
        for k in 1:5 loop
          if k > i then
            factor := A[k, i]/A[i, i];
            for j in 1:5 loop
              A[k, j] := A[k, j] - factor*A[i, j];
            end for;
            b[k] := b[k] - factor*b[i];
          end if;
        end for;
      end for;
      c := zeros(5);
      for m in 1:5 loop
        c[6 - m] := (b[6 - m] - sum((if j > 6 - m then A[6 - m, j]*c[j] else 0) for j in 1:5))/A[6 - m, 6 - m];
      end for;
    end polynomialFit;

    function equalPercentage
      "Equal-percentage valve characteristic with leakage (Buildings.Fluid.Actuators.BaseClasses.equalPercentage):
       linear below delta0/2, R^(y-1) above 1.5*delta0, a cubic Hermite joining them"
      input Real y "Opening";
      input Real R "Rangeability";
      input Real l "Leakage at y = 0";
      input Real delta0 "Width of the linear range near closure";
      output Real phi;
    protected
      Real x0 = delta0/2;
      Real x1 = 1.5*delta0;
      Real y0 = l + x0*(R^(delta0 - 1) - l)/delta0;
      Real s0 = (R^(delta0 - 1) - l)/delta0;
      Real y1 = R^(x1 - 1);
      Real s1 = log(R)*R^(x1 - 1);
      Real h;
      Real u;
    algorithm
      if y < x0 then
        phi := l + y*(R^(delta0 - 1) - l)/delta0;
      elseif y > x1 then
        phi := R^(y - 1);
      else
        h := x1 - x0;
        u := (y - x0)/h;
        phi := (2*u^3 - 3*u^2 + 1)*y0 + (u^3 - 2*u^2 + u)*h*s0 + (-2*u^3 + 3*u^2)*y1 + (u^3 - u^2)*h*s1;
      end if;
    end equalPercentage;

    function valveConductance
      "Conductance m_flow/dp of a valve with a fixed resistance in series, its flow law linearized as Buildings does
       (linearized=true): m_flow*m_flow_nominal = k^2*dp, 1/k^2 = 1/kFixed^2 + 1/(phi*Kv)^2"
      input Real phi "Valve characteristic";
      input Real m_flow_nominal(unit="kg/s");
      input Real dpValve_nominal(unit="Pa");
      input Real dpFixed_nominal(unit="Pa");
      output Real G(unit="kg/(s.Pa)");
    algorithm
      G := 1/(m_flow_nominal*(dpFixed_nominal/m_flow_nominal^2 + dpValve_nominal/(phi*m_flow_nominal)^2));
    end valveConductance;

    function quadraticFlow
      "Flow through a quadratic resistance (m_flow_nominal at dp_nominal), regularized below deltaM*m_flow_nominal"
      input Real dp(unit="Pa");
      input Real m_flow_nominal(unit="kg/s");
      input Real dp_nominal(unit="Pa");
      input Real deltaM = 0.1;
      output Real m_flow(unit="kg/s");
    algorithm
      m_flow := m_flow_nominal/sqrt(dp_nominal)*dp/(dp^2 + (deltaM^2*dp_nominal)^2)^0.25;
    end quadraticFlow;

    function timeOfDay "Seconds since midnight, without events"
      input Real t(unit="s");
      output Real tod(unit="s");
    algorithm
      tod := noEvent(t - 86400*floor(t/86400));
    end timeOfDay;

    function occupancy
      "1 between the pairs of instants of a day schedule {on, off, on, off, ...} [s] (Buildings OccupancySchedule
       with firstEntryOccupied), each change a smooth step over dt"
      input Real tod(unit="s") "Time of day";
      input Real schedule[:](each unit="s");
      input Real dt(unit="s") = 60;
      output Real y;
    algorithm
      y := 0;
      for k in 1:div(size(schedule, 1), 2) loop
        y := y + regStep(tod - schedule[2*k - 1], 1, 0, dt/2) - regStep(tod - schedule[2*k], 1, 0, dt/2);
      end for;
    end occupancy;

    function dayTable "Linear interpolation in a day table [time of day (s), value], repeated daily"
      input Real tod(unit="s");
      input Real table[:, 2];
      output Real y;
    protected
      Integer n = size(table, 1);
      Real w;
    algorithm
      y := table[1, 2];
      for i in 1:n - 1 loop
        if tod > table[i, 1] then
          w := min((tod - table[i, 1])/max(table[i + 1, 1] - table[i, 1], 1e-6), 1);
          y := table[i, 2] + w*(table[i + 1, 2] - table[i, 2]);
        end if;
      end for;
      if n > 1 and tod > table[n, 1] then
        y := table[n, 2] + (tod - table[n, 1])/(86400 + table[1, 1] - table[n, 1])*(table[1, 2] - table[n, 2]);
      end if;
    end dayTable;

    function polynomial "c[1] + c[2]*x + ... + c[n]*x^(n-1)"
      input Real c[:];
      input Real x;
      output Real y;
    algorithm
      y := sum(c[i]*x^(i - 1) for i in 1:size(c, 1));
    end polynomial;
  end Utilities;

  package Interfaces "Connectors"
    connector RealInput = input Real "Real input signal"
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}), graphics={
          Polygon(lineColor={0,0,127}, fillColor={0,0,127}, fillPattern=FillPattern.Solid,
            points={{-100,100},{100,0},{-100,-100}})}));
    connector RealOutput = output Real "Real output signal"
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}), graphics={
          Polygon(lineColor={0,0,127}, fillColor={255,255,255}, fillPattern=FillPattern.Solid,
            points={{-100,100},{100,0},{-100,-100}})}));

    connector HeatPort "Thermal port: temperature and heat flow rate into the component"
      Real T(unit="K") "Port temperature";
      flow Real Q_flow(unit="W") "Heat flow rate into the component";
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={191,0,0}, fillColor={191,0,0},
            fillPattern=FillPattern.Solid)}));
    end HeatPort;

    connector WaterInlet "Water inlet: mass flow rate into the component, temperature of the water crossing it"
      flow Real m_flow(unit="kg/s") "Mass flow rate into the component";
      Real T(unit="K") "Temperature of the water crossing the port (set by the component the water leaves)";
      RealOutput p(unit="Pa") "Pressure of the inlet node of the component (0 with set flows)";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Ellipse(extent={{-100,100},{100,-100}}, lineColor={0,127,255}, fillColor={0,127,255},
            fillPattern=FillPattern.Solid)}));
    end WaterInlet;

    connector WaterOutlet "Water outlet: mass flow rate into the component, temperature of the water crossing it"
      flow Real m_flow(unit="kg/s") "Mass flow rate into the component (negative when water leaves)";
      Real T(unit="K") "Temperature of the water crossing the port (set by the component)";
      RealInput p(unit="Pa") "Pressure of the inlet node of the component downstream";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Ellipse(extent={{-100,100},{100,-100}}, lineColor={0,127,255}, fillColor={0,127,255},
            fillPattern=FillPattern.Solid),
          Ellipse(extent={{-80,80},{80,-80}}, lineColor={0,127,255}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
    end WaterOutlet;

    connector WeatherInputs "Weather data: inputs of building_mpc (from a weather file or a forecast)"
      RealInput TDryBul(unit="K") "Outdoor dry-bulb air temperature";
      RealInput TBlaSky(unit="K") "Black-body sky temperature";
      RealInput winSpe(unit="m/s") "Wind speed";
      RealInput winDir(unit="rad") "Wind direction, 0 from the north";
      RealInput HDirNor(unit="W/m2") "Direct normal irradiance";
      RealInput HDifHor(unit="W/m2") "Diffuse horizontal irradiance";
      RealInput HGloHor(unit="W/m2") "Global horizontal irradiance";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(lineColor={255,204,51}, lineThickness=0.5, extent={{-20,-2},{20,2}}),
          Polygon(fillColor={255,215,136}, fillPattern=FillPattern.Solid,
            points={{-80,50},{80,50},{100,30},{80,-40},{60,-50},{-60,-50},{-80,-40},{-100,30}},
            smooth=Smooth.Bezier),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-65,15},{-55,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-5,15},{5,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{55,15},{65,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-35,-25},{-25,-15}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{25,-25},{35,-15}})}));
    end WeatherInputs;

    connector WeatherBus "Weather data and solar position (source side)"
      RealOutput TDryBul(unit="K") "Outdoor dry-bulb air temperature";
      RealOutput TBlaSky(unit="K") "Black-body sky temperature";
      RealOutput winSpe(unit="m/s") "Wind speed";
      RealOutput winDir(unit="rad") "Wind direction, 0 from the north";
      RealOutput HDirNor(unit="W/m2") "Direct normal irradiance";
      RealOutput HDifHor(unit="W/m2") "Diffuse horizontal irradiance";
      RealOutput HGloHor(unit="W/m2") "Global horizontal irradiance";
      RealOutput lat(unit="rad") "Latitude";
      RealOutput decAng(unit="rad") "Solar declination";
      RealOutput solHouAng(unit="rad") "Solar hour angle";
      RealOutput cosZen "Cosine of the solar zenith angle";
      RealOutput F1 "Circumsolar brightening coefficient of the Perez model";
      RealOutput F2 "Horizon brightening coefficient of the Perez model";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(lineColor={255,204,51}, lineThickness=0.5, extent={{-20,-2},{20,2}}),
          Polygon(fillColor={255,215,136}, fillPattern=FillPattern.Solid,
            points={{-80,50},{80,50},{100,30},{80,-40},{60,-50},{-60,-50},{-80,-40},{-100,30}},
            smooth=Smooth.Bezier),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-65,15},{-55,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-5,15},{5,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{55,15},{65,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-35,-25},{-25,-15}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{25,-25},{35,-15}})}));
    end WeatherBus;

    connector WeatherBusIn "Weather data and solar position (receiving side)"
      RealInput TDryBul(unit="K") "Outdoor dry-bulb air temperature";
      RealInput TBlaSky(unit="K") "Black-body sky temperature";
      RealInput winSpe(unit="m/s") "Wind speed";
      RealInput winDir(unit="rad") "Wind direction, 0 from the north";
      RealInput HDirNor(unit="W/m2") "Direct normal irradiance";
      RealInput HDifHor(unit="W/m2") "Diffuse horizontal irradiance";
      RealInput HGloHor(unit="W/m2") "Global horizontal irradiance";
      RealInput lat(unit="rad") "Latitude";
      RealInput decAng(unit="rad") "Solar declination";
      RealInput solHouAng(unit="rad") "Solar hour angle";
      RealInput cosZen "Cosine of the solar zenith angle";
      RealInput F1 "Circumsolar brightening coefficient of the Perez model";
      RealInput F2 "Horizon brightening coefficient of the Perez model";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(lineColor={255,204,51}, lineThickness=0.5, extent={{-20,-2},{20,2}}),
          Polygon(fillColor={255,215,136}, fillPattern=FillPattern.Solid,
            points={{-80,50},{80,50},{100,30},{80,-40},{60,-50},{-60,-50},{-80,-40},{-100,30}},
            smooth=Smooth.Bezier),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-65,15},{-55,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-5,15},{5,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{55,15},{65,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-35,-25},{-25,-15}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{25,-25},{35,-15}})}));
    end WeatherBusIn;

    connector SurfaceBus "A surface of a zone, surface side: heat exchanges, properties, weather"
      HeatPort con "Convection with the zone air (at the air temperature)";
      HeatPort rad "Long-wave and solar exchange with the room (at the surface temperature)";
      RealOutput A(unit="m2") "Area seen from the room";
      RealOutput epsLw "Long-wave emissivity of the room side";
      RealOutput absSol "Solar absorptance of the room side (diffuse)";
      RealOutput tauSol "Solar transmittance back to the outside (windows)";
      RealOutput isFloor "1 for a floor, which receives the transmitted direct radiation first";
      RealOutput QTraDir(unit="W") "Direct solar radiation transmitted into the room (windows)";
      RealOutput QTraDif(unit="W") "Diffuse solar radiation transmitted into the room (windows)";
      RealInput hConInt(unit="W/(m2.K)") "Fixed interior convection coefficient, 0 for the natural convection law";
      RealInput hConExt(unit="W/(m2.K)") "Fixed exterior convection coefficient, 0 for the wind-dependent law";
      WeatherBusIn wea "Weather";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(lineColor={255,204,51}, lineThickness=0.5, extent={{-20,-2},{20,2}}),
          Polygon(fillColor={255,215,136}, fillPattern=FillPattern.Solid,
            points={{-80,50},{80,50},{100,30},{80,-40},{60,-50},{-60,-50},{-80,-40},{-100,30}},
            smooth=Smooth.Bezier),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-65,15},{-55,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-5,15},{5,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{55,15},{65,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-35,-25},{-25,-15}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{25,-25},{35,-15}})}));
    end SurfaceBus;

    connector ZoneSurfaceBus "A surface of a zone, zone side"
      HeatPort con "Convection with the zone air (at the air temperature)";
      HeatPort rad "Long-wave and solar exchange with the room (at the surface temperature)";
      RealInput A(unit="m2") "Area seen from the room";
      RealInput epsLw "Long-wave emissivity of the room side";
      RealInput absSol "Solar absorptance of the room side (diffuse)";
      RealInput tauSol "Solar transmittance back to the outside (windows)";
      RealInput isFloor "1 for a floor, which receives the transmitted direct radiation first";
      RealInput QTraDir(unit="W") "Direct solar radiation transmitted into the room (windows)";
      RealInput QTraDif(unit="W") "Diffuse solar radiation transmitted into the room (windows)";
      RealOutput hConInt(unit="W/(m2.K)") "Fixed interior convection coefficient, 0 for the natural convection law";
      RealOutput hConExt(unit="W/(m2.K)") "Fixed exterior convection coefficient, 0 for the wind-dependent law";
      WeatherBus wea "Weather";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(lineColor={255,204,51}, lineThickness=0.5, extent={{-20,-2},{20,2}}),
          Polygon(fillColor={255,215,136}, fillPattern=FillPattern.Solid,
            points={{-80,50},{80,50},{100,30},{80,-40},{60,-50},{-60,-50},{-80,-40},{-100,30}},
            smooth=Smooth.Bezier),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-65,15},{-55,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-5,15},{5,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{55,15},{65,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-35,-25},{-25,-15}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{25,-25},{35,-15}})}));
    end ZoneSurfaceBus;

    connector ControlInput "A control of building_mpc: input between its bounds (a decision of the MPC)"
      RealInput y(min=0, max=1, unit="1") "Control signal, 0 to 1";
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}), graphics={
          Polygon(lineColor={0,0,127}, fillColor={0,0,127}, fillPattern=FillPattern.Solid,
            points={{-100,100},{100,0},{-100,-100}})}));
    end ControlInput;

    connector ControlSignal "Control signal of a component (receiving side)"
      RealInput y(unit="1") "Control signal, 0 to 1";
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}), graphics={
          Polygon(lineColor={0,0,127}, fillColor={0,0,127}, fillPattern=FillPattern.Solid,
            points={{-100,100},{100,0},{-100,-100}})}));
    end ControlSignal;

    connector GainInputs "Internal gains per floor area: inputs (from an occupancy schedule)"
      RealInput QCon(unit="W/m2") "Convective internal gains per floor area";
      RealInput QRad(unit="W/m2") "Radiant internal gains per floor area";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(lineColor={255,204,51}, lineThickness=0.5, extent={{-20,-2},{20,2}}),
          Polygon(fillColor={255,215,136}, fillPattern=FillPattern.Solid,
            points={{-80,50},{80,50},{100,30},{80,-40},{60,-50},{-60,-50},{-80,-40},{-100,30}},
            smooth=Smooth.Bezier),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-65,15},{-55,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-5,15},{5,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{55,15},{65,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-35,-25},{-25,-15}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{25,-25},{35,-15}})}));
    end GainInputs;

    connector OccupancyInputs "Internal gains of a zone: inputs of building_mpc"
      GainInputs gai "Internal gains per floor area";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(lineColor={255,204,51}, lineThickness=0.5, extent={{-20,-2},{20,2}}),
          Polygon(fillColor={255,215,136}, fillPattern=FillPattern.Solid,
            points={{-80,50},{80,50},{100,30},{80,-40},{60,-50},{-60,-50},{-80,-40},{-100,30}},
            smooth=Smooth.Bezier),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-65,15},{-55,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-5,15},{5,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{55,15},{65,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-35,-25},{-25,-15}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{25,-25},{35,-15}})}));
    end OccupancyInputs;

    connector Gains "Internal gains per floor area (receiving side)"
      RealInput QCon(unit="W/m2") "Convective internal gains per floor area";
      RealInput QRad(unit="W/m2") "Radiant internal gains per floor area";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(lineColor={255,204,51}, lineThickness=0.5, extent={{-20,-2},{20,2}}),
          Polygon(fillColor={255,215,136}, fillPattern=FillPattern.Solid,
            points={{-80,50},{80,50},{100,30},{80,-40},{60,-50},{-60,-50},{-80,-40},{-100,30}},
            smooth=Smooth.Bezier),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-65,15},{-55,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-5,15},{5,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{55,15},{65,25}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{-35,-25},{-25,-15}}),
          Ellipse(fillPattern=FillPattern.Solid, extent={{25,-25},{35,-15}})}));
    end Gains;

    connector HeatFlowInput "Heat flow rate of an ideal system: input of building_mpc"
      RealInput Q_flow(unit="W") "Heat flow rate, positive for heating";
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}), graphics={
          Polygon(lineColor={0,0,127}, fillColor={0,0,127}, fillPattern=FillPattern.Solid,
            points={{-100,100},{100,0},{-100,-100}})}));
    end HeatFlowInput;
  end Interfaces;

  package Data "Records of constructions and glazings, written by trano from the YAML file"
    record Construction "Opaque construction: layers from the outside (side a) to the inside (side b)"
      parameter Integer nLay "Number of layers";
      parameter Real x[nLay](each unit="m") "Thickness";
      parameter Real k[nLay](each unit="W/(m.K)") "Thermal conductivity";
      parameter Real c[nLay](each unit="J/(kg.K)") "Specific heat capacity";
      parameter Real d[nLay](each unit="kg/m3") "Density";
      parameter Real absIR_a = 0.84 "Long-wave emissivity of side a (outside)";
      parameter Real absIR_b = 0.84 "Long-wave emissivity of side b (inside)";
      parameter Real absSol_a = 0.84 "Solar absorptance of side a (outside)";
      parameter Real absSol_b = 0.84 "Solar absorptance of side b (inside)";
    end Construction;

    record Glazing "Glazing: panes from the outside, angular optics at the incidence angles 0, 10, ..., 90 degrees"
      parameter Integer nPan "Number of panes";
      parameter Real x[nPan](each unit="m") "Thickness of the panes";
      parameter Real d[nPan](each unit="kg/m3") "Density of the panes";
      parameter Real c[nPan](each unit="J/(kg.K)") "Specific heat capacity of the panes";
      parameter Real RPan[nPan](each unit="m2.K/W") "Thermal resistance of each pane";
      parameter Real RGap[nPan - 1](each unit="m2.K/W") "Thermal resistance of each gas gap (EN 673)";
      parameter Real tau[11] "Solar transmittance at the 10 incidence angles, then hemispherical";
      parameter Real absPan[nPan, 11] "Solar absorptance of each pane at the 10 incidence angles, then hemispherical";
      parameter Real absIR_a "Long-wave emissivity of the outer pane, outside face";
      parameter Real absIR_b "Long-wave emissivity of the inner pane, room face";
      parameter Real UFra(unit="W/(m2.K)") = 1.4 "Thermal transmittance of the frame";
    end Glazing;
  end Data;

  package BoundaryConditions "Weather and solar irradiance (after Buildings.BoundaryConditions)"
    model Weather "Weather data, solar position and Perez coefficients for the site"
      parameter Real lat(unit="rad") = 0.8868 "Latitude (Uccle by default)";
      parameter Real lon(unit="rad") = 0.0760 "Longitude (Uccle by default)";
      parameter Real timZon(unit="s") = 3600 "Time zone (offset of the standard time from UTC)";
      parameter Real alt(unit="m") = 100 "Altitude of the site";
      Interfaces.RealInput TDryBul(unit="K") "Outdoor dry-bulb air temperature"
        annotation (Placement(transformation(extent={{-120,60},{-100,80}})));
      Interfaces.RealInput TBlaSky(unit="K") "Black-body sky temperature"
        annotation (Placement(transformation(extent={{-120,40},{-100,60}})));
      Interfaces.RealInput winSpe(unit="m/s") "Wind speed"
        annotation (Placement(transformation(extent={{-120,20},{-100,40}})));
      Interfaces.RealInput winDir(unit="rad") "Wind direction, 0 from the north"
        annotation (Placement(transformation(extent={{-120,0},{-100,20}})));
      Interfaces.RealInput HDirNor(unit="W/m2") "Direct normal irradiance"
        annotation (Placement(transformation(extent={{-120,-20},{-100,0}})));
      Interfaces.RealInput HDifHor(unit="W/m2") "Diffuse horizontal irradiance"
        annotation (Placement(transformation(extent={{-120,-40},{-100,-20}})));
      Interfaces.RealInput HGloHor(unit="W/m2") "Global horizontal irradiance"
        annotation (Placement(transformation(extent={{-120,-60},{-100,-40}})));
      Interfaces.WeatherBus weaBus "Weather data and solar position"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    protected
      Real Bt "Intermediate variable of the equation of time";
      Real eqnTim(unit="s") "Equation of time";
      Real solTim(unit="s") "Solar time";
      Real zen(unit="rad") "Solar zenith angle";
      Real zenLim(unit="rad") "Zenith angle bounded by 90 degrees";
      Real relAirMas "Relative air mass";
      Real skyBri "Sky brightness";
      Real HDifHorBou(unit="W/m2") "Diffuse horizontal irradiance bounded away from 0";
      Real tmp "Intermediate variable of the sky clearness";
      Real skyCle "Sky clearness";
      Real a[8] "Weights of the sky clearness bins";
      Real b[8] "Smooth steps of the sky clearness bins";
      constant Real edges[7] = {1.065, 1.23, 1.50, 1.95, 2.80, 4.50, 6.20} "Sky clearness bin edges";
      constant Real F11[8] = {-0.0083117, 0.1299457, 0.3296958, 0.5682053, 0.8730280, 1.1326077, 1.0601591, 0.6777470};
      constant Real F12[8] = {0.5877285, 0.6825954, 0.4868735, 0.1874525, -0.3920403, -1.2367284, -1.5999137, -0.3272588};
      constant Real F13[8] = {-0.0620636, -0.1513725, -0.2210958, -0.2951290, -0.3616149, -0.4118494, -0.3589221, -0.2504286};
      constant Real F21[8] = {-0.0596012, -0.0189325, 0.0554140, 0.1088631, 0.2255647, 0.2877813, 0.2642124, 0.1561313};
      constant Real F22[8] = {0.0721249, 0.0659650, -0.0639588, -0.1519229, -0.4620442, -0.8230357, -1.1272340, -1.3765031};
      constant Real F23[8] = {-0.0220216, -0.0288748, -0.0260542, -0.0139754, 0.0012448, 0.0558651, 0.1310694, 0.2506212};
    equation
      // Weather data
      weaBus.TDryBul = TDryBul;
      weaBus.TBlaSky = TBlaSky;
      weaBus.winSpe = winSpe;
      weaBus.winDir = winDir;
      weaBus.HDirNor = HDirNor;
      weaBus.HDifHor = HDifHor;
      weaBus.HGloHor = HGloHor;
      weaBus.lat = lat;
      // Solar position (Buildings.BoundaryConditions.SolarGeometry; the model time is the clock time of the year)
      weaBus.decAng = asin(-sin(23.45*pi/180)*cos((time/86400 + 10)*2*pi/365.25));
      Bt = pi*((time + 86400)/86400 - 81)/182;
      eqnTim = 60*(9.87*sin(2*Bt) - 7.53*cos(Bt) - 1.5*sin(Bt));
      solTim = time - timZon + lon*43200/pi + eqnTim;
      weaBus.solHouAng = (solTim/3600 - 12)*2*pi/24;
      weaBus.cosZen = cos(lat)*cos(weaBus.decAng)*cos(weaBus.solHouAng) + sin(lat)*sin(weaBus.decAng);
      zen = acos(Utilities.smoothLimit(weaBus.cosZen, -1, 1, 1e-6));
      // Perez sky model (Buildings.BoundaryConditions.SolarIrradiation.BaseClasses)
      zenLim = Utilities.smoothMin(zen, pi/2, 0.01);
      relAirMas = exp(-0.0001184*alt)/(cos(zenLim) + 0.15*(93.9 - zenLim*180/pi)^(-1.253));
      skyBri = Utilities.smoothMin(HDifHor*relAirMas/(1366.1*(1 + 0.033*cos(2*pi*(solTim/86400 + 1)/365))), 1, 0.025);
      HDifHorBou = Utilities.smoothMax(HDifHor, 0.5e-4, 0.25e-4);
      tmp = 5.534e-6*(180/pi)^3*zen^3;
      skyCle = Utilities.smoothLimit(((HDirNor + HDifHorBou)/HDifHorBou + tmp)/(1 + tmp), 1, 8, 0.01);
      for i in 1:7 loop
        b[i] = Utilities.regStep(edges[i] - skyCle, 1, 0, 0.01);
      end for;
      b[8] = Utilities.regStep(skyCle - edges[7], 1, 0, 0.01);
      a[1] = b[1];
      for i in 2:7 loop
        a[i] = b[i] - b[i - 1];
      end for;
      a[8] = b[8];
      weaBus.F1 = Utilities.smoothMax(0, sum(a[i]*(F11[i] + F12[i]*skyBri + F13[i]*zen) for i in 1:8), 0.01);
      weaBus.F2 = sum(a[i]*(F21[i] + F22[i]*skyBri + F23[i]*zen) for i in 1:8);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={124,142,255}, fillColor={124,142,255},
            fillPattern=FillPattern.Solid),
          Ellipse(extent={{-73,77},{14,-10}}, lineColor={255,220,220}, lineThickness=1,
            fillPattern=FillPattern.Sphere, fillColor={255,255,0}),
          Polygon(points={{47,53}, {39,21}, {55,26}, {69,25}, {82,19}, {91,-14}, {69,-51}, {5,-55}, {-70,-53},
            {-83,-15}, {-75,12}, {-51,13}, {-39,4}, {-46,35}, {-29,60}, {17,70}, {47,53}}, lineColor={220,220,220},
            lineThickness=0.1, fillPattern=FillPattern.Sphere, smooth=Smooth.Bezier, fillColor={230,230,230}),
          Text(extent={{-98,78},{-60,62}}, textColor={0,0,0}, textString="TDryBul"),
          Text(extent={{-98,58},{-60,42}}, textColor={0,0,0}, textString="TBlaSky"),
          Text(extent={{-98,38},{-60,22}}, textColor={0,0,0}, textString="winSpe"),
          Text(extent={{-98,18},{-60,2}}, textColor={0,0,0}, textString="winDir"),
          Text(extent={{-98,-2},{-60,-18}}, textColor={0,0,0}, textString="HDirNor"),
          Text(extent={{-98,-22},{-60,-38}}, textColor={0,0,0}, textString="HDifHor"),
          Text(extent={{-98,-42},{-60,-58}}, textColor={0,0,0}, textString="HGloHor"),
          Text(extent={{-150,140},{150,100}}, textColor={0,0,255}, textString="%name")}));
    end Weather;

    model TiltedSurface "Solar irradiance on a tilted surface: direct, Perez diffuse (sky and ground), incidence"
      parameter Real azi(unit="rad") "Surface azimuth: 0 south, pi/2 west";
      parameter Real til(unit="rad") "Surface tilt: 0 roof, pi/2 wall";
      parameter Real rho = 0.2 "Ground reflectance";
      Interfaces.WeatherBusIn weaBus
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Real cosInc "Cosine of the incidence angle";
      Real HDir(unit="W/m2") "Direct irradiance on the surface";
      Real HDif(unit="W/m2") "Diffuse irradiance on the surface (sky and ground)";
    protected
      constant Real bMin = cos(85*pi/180);
    equation
      cosInc = cos(til)*(cos(weaBus.decAng)*cos(weaBus.solHouAng)*cos(weaBus.lat) + sin(weaBus.decAng)*sin(weaBus.lat))
        + sin(til)*(sin(azi)*cos(weaBus.decAng)*sin(weaBus.solHouAng) + cos(azi)*(cos(weaBus.decAng)
        *cos(weaBus.solHouAng)*sin(weaBus.lat) - sin(weaBus.decAng)*cos(weaBus.lat)));
      HDir = Utilities.positivePart(cosInc*weaBus.HDirNor, 1e-2);
      HDif = weaBus.HDifHor*(0.5*(1 - weaBus.F1)*(1 + cos(til)) + weaBus.F1*Utilities.smoothMax(0, cosInc, 0.01)
        /Utilities.smoothMax(bMin, weaBus.cosZen, 0.01) + weaBus.F2*sin(til)) + weaBus.HGloHor*0.5*rho*(1 - cos(til));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Ellipse(extent={{-80,80},{-20,20}}, lineColor={255,220,220}, fillColor={255,255,0},
            fillPattern=FillPattern.Sphere),
          Polygon(points={{-20,-80},{80,20},{80,-80},{-20,-80}}, lineColor={0,0,0}, fillColor={175,175,175},
            fillPattern=FillPattern.Solid),
          Line(points={{-30,30},{30,-30}}, color={255,128,0}, arrow={Arrow.None,Arrow.Filled}),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end TiltedSurface;
  end BoundaryConditions;

  package Envelope "Walls, floors, roofs and windows (after Buildings.ThermalZones.Detailed and Buildings.HeatTransfer)"
    partial model PartialSurface "A surface of a zone: area, orientation and its bus"
      parameter Real A(unit="m2") "Area seen from the room";
      parameter Real azi(unit="rad") = 0 "Azimuth: 0 south, pi/2 west";
      parameter Real til(unit="rad") "Tilt: 0 roof (ceiling), pi/2 wall, pi floor";
      parameter Real T_start(unit="K") = 294.15 "Initial temperature";
      Interfaces.SurfaceBus propsBus_a "Connection to the zone"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    protected
      parameter Real cIntPos = Utilities.convectionCoefficient(til, true) "Interior natural convection, surface warmer";
      parameter Real cIntNeg = Utilities.convectionCoefficient(til, false) "Interior natural convection, surface colder";
      parameter Real isFloor = if abs(sin(til)) < 1e-6 and cos(til) < 0 then 1 else 0;
    end PartialSurface;

    partial model PartialOpaque "Conduction through the layers of a construction, one node at each layer boundary"
      extends PartialSurface;
      parameter Data.Construction constructionType "Construction, layers from the outside";
      parameter Boolean stateAtSurface_a = true "Capacity at the outer surface (false: it is held at a temperature)";
    protected
      parameter Integer nLay = constructionType.nLay;
      parameter Real C[nLay + 1](each unit="J/K") = {A*((if i > 1 then constructionType.x[i - 1]*constructionType.d[i - 1]
        *constructionType.c[i - 1]/2 else 0) + (if i <= nLay then constructionType.x[i]*constructionType.d[i]
        *constructionType.c[i]/2 else 0)) for i in 1:nLay + 1} "Capacity of each node";
      parameter Real G[nLay](each unit="W/K") = {A*constructionType.k[i]/constructionType.x[i] for i in 1:nLay}
        "Conductance of each layer";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{0,80},{80,-80}}, fillColor={175,175,175}, fillPattern=FillPattern.Solid,
            lineColor={175,175,175}),
          Rectangle(extent={{-80,80},{0,-80}}, fillColor={215,215,215}, fillPattern=FillPattern.Solid,
            lineColor={175,175,175}),
          Line(points={{-92,0},{90,0}}, color={0,0,0}, thickness=0.5),
          Line(points={{-18,-40},{-32,-40}}, color={0,0,0}, thickness=0.5),
          Line(points={{-12,-32},{-38,-32}}, color={0,0,0}, thickness=0.5),
          Line(points={{-25,0},{-25,-32}}, color={0,0,0}, thickness=0.5),
          Line(points={{32,-40},{18,-40}}, color={0,0,0}, thickness=0.5),
          Line(points={{38,-32},{12,-32}}, color={0,0,0}, thickness=0.5),
          Line(points={{25,0},{25,-32}}, color={0,0,0}, thickness=0.5),
          Rectangle(extent={{-60,6},{-40,-6}}, lineColor={0,0,0}, lineThickness=0.5, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{-10,6},{10,-6}}, lineColor={0,0,0}, lineThickness=0.5, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{40,6},{60,-6}}, lineColor={0,0,0}, lineThickness=0.5, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end PartialOpaque;

    model OuterWall "Exterior wall or roof: wind and sky at the outside, sun, conduction, room exchanges"
      extends PartialOpaque;
      Real T[nLay + 1](each unit="K", each start=T_start, each fixed=true) "Node temperatures, outside first";
      BoundaryConditions.TiltedSurface sol(final azi=azi, final til=til) "Irradiance on the outer surface"
        annotation (Placement(transformation(extent={{-70,60},{-50,80}})));
      Real QSol(unit="W") "Absorbed solar radiation";
      Real QExt(unit="W") "Heat from the outer surface to the outside air and the sky";
      Real QInt(unit="W") "Heat from the inner surface to the room air";
    protected
      parameter Real cExtPos = Utilities.convectionCoefficient(pi - til, true) "Outer face: tilt pi - til";
      parameter Real cExtNeg = Utilities.convectionCoefficient(pi - til, false);
    equation
      connect(propsBus_a.wea, sol.weaBus);
      QSol = constructionType.absSol_a*A*(sol.HDir + sol.HDif);
      QExt = Functions.exteriorExchange(T[1], A, til, azi, cExtPos, cExtNeg, constructionType.absIR_a, propsBus_a.hConExt,
        propsBus_a.wea.TDryBul, propsBus_a.wea.TBlaSky, propsBus_a.wea.winSpe, propsBus_a.wea.winDir);
      QInt = Functions.interiorConvection(T[nLay + 1] - propsBus_a.con.T, A, cIntPos, cIntNeg, propsBus_a.hConInt);
      C[1]*der(T[1]) = QSol - QExt - G[1]*(T[1] - T[2]);
      for i in 2:nLay loop
        C[i]*der(T[i]) = G[i - 1]*(T[i - 1] - T[i]) - G[i]*(T[i] - T[i + 1]);
      end for;
      C[nLay + 1]*der(T[nLay + 1]) = G[nLay]*(T[nLay] - T[nLay + 1]) - QInt + propsBus_a.rad.Q_flow;
      propsBus_a.con.Q_flow = -QInt;
      propsBus_a.rad.T = T[nLay + 1];
      propsBus_a.A = A;
      propsBus_a.epsLw = constructionType.absIR_b;
      propsBus_a.absSol = constructionType.absSol_b;
      propsBus_a.tauSol = 0;
      propsBus_a.isFloor = isFloor;
      propsBus_a.QTraDir = 0;
      propsBus_a.QTraDif = 0;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Ellipse(extent={{-100,100},{-68,68}}, lineColor={255,255,0}, fillColor={255,213,170},
            fillPattern=FillPattern.Sphere)}));
    end OuterWall;

    model BoundaryWall "Floor on ground or over outdoor air: the outer surface is held at a temperature"
      extends PartialOpaque(final stateAtSurface_a=false);
      parameter Boolean outdoorAir = false "true: the outer surface at the outdoor air temperature";
      parameter Real TGro(unit="K") = 283.15 "Ground temperature";
      Real T[nLay](each unit="K", each start=T_start, each fixed=true) "Node temperatures, the outer surface excluded";
      Real QInt(unit="W") "Heat from the inner surface to the room air";
    protected
      Real TNod[nLay + 1](each unit="K") "Node temperatures, the outer surface first";
    equation
      TNod[1] = if outdoorAir then propsBus_a.wea.TDryBul else TGro;
      for i in 1:nLay loop
        TNod[i + 1] = T[i];
      end for;
      QInt = Functions.interiorConvection(T[nLay] - propsBus_a.con.T, A, cIntPos, cIntNeg, propsBus_a.hConInt);
      for i in 1:nLay - 1 loop
        C[i + 1]*der(T[i]) = G[i]*(TNod[i] - T[i]) - G[i + 1]*(T[i] - T[i + 1]);
      end for;
      C[nLay + 1]*der(T[nLay]) = G[nLay]*(TNod[nLay] - T[nLay]) - QInt + propsBus_a.rad.Q_flow;
      propsBus_a.con.Q_flow = -QInt;
      propsBus_a.rad.T = T[nLay];
      propsBus_a.A = A;
      propsBus_a.epsLw = constructionType.absIR_b;
      propsBus_a.absSol = constructionType.absSol_b;
      propsBus_a.tauSol = 0;
      propsBus_a.isFloor = isFloor;
      propsBus_a.QTraDir = 0;
      propsBus_a.QTraDif = 0;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,-80},{-80,-100}}, lineColor={127,0,0}, fillColor={127,0,0},
            fillPattern=FillPattern.Backward)}));
    end BoundaryWall;

    model AdiabaticWall "Wall without heat flow through its outer surface"
      extends PartialOpaque;
      Real T[nLay + 1](each unit="K", each start=T_start, each fixed=true) "Node temperatures, outside first";
      Real QInt(unit="W") "Heat from the inner surface to the room air";
    equation
      QInt = Functions.interiorConvection(T[nLay + 1] - propsBus_a.con.T, A, cIntPos, cIntNeg, propsBus_a.hConInt);
      C[1]*der(T[1]) = -G[1]*(T[1] - T[2]);
      for i in 2:nLay loop
        C[i]*der(T[i]) = G[i - 1]*(T[i - 1] - T[i]) - G[i]*(T[i] - T[i + 1]);
      end for;
      C[nLay + 1]*der(T[nLay + 1]) = G[nLay]*(T[nLay] - T[nLay + 1]) - QInt + propsBus_a.rad.Q_flow;
      propsBus_a.con.Q_flow = -QInt;
      propsBus_a.rad.T = T[nLay + 1];
      propsBus_a.A = A;
      propsBus_a.epsLw = constructionType.absIR_b;
      propsBus_a.absSol = constructionType.absSol_b;
      propsBus_a.tauSol = 0;
      propsBus_a.isFloor = isFloor;
      propsBus_a.QTraDir = 0;
      propsBus_a.QTraDif = 0;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Line(points={{-90,80},{-90,-80}}, color={0,0,0}, thickness=1)}));
    end AdiabaticWall;

    model InternalWall "Wall or floor between two zones: side a faces the zone of propsBus_a, side b the other"
      extends PartialOpaque;
      parameter Real epsLw = 0.84 "Long-wave emissivity of both faces (Buildings surBou)";
      parameter Real absSol = 0.84 "Solar absorptance of both faces (Buildings surBou)";
      Interfaces.SurfaceBus propsBus_b "Connection to the zone of side b"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Real T[nLay + 1](each unit="K", each start=T_start, each fixed=true) "Node temperatures, side a first";
      Real QInt_a(unit="W") "Heat from side a to the air of its zone";
      Real QInt_b(unit="W") "Heat from side b to the air of its zone";
    protected
      parameter Real cPos_b = Utilities.convectionCoefficient(pi - til, true) "Side b faces the other way";
      parameter Real cNeg_b = Utilities.convectionCoefficient(pi - til, false);
      parameter Real isFloor_b = if abs(sin(til)) < 1e-6 and cos(til) > 0 then 1 else 0;
    equation
      QInt_a = Functions.interiorConvection(T[1] - propsBus_a.con.T, A, cIntPos, cIntNeg, propsBus_a.hConInt);
      QInt_b = Functions.interiorConvection(T[nLay + 1] - propsBus_b.con.T, A, cPos_b, cNeg_b, propsBus_b.hConInt);
      C[1]*der(T[1]) = -G[1]*(T[1] - T[2]) - QInt_a + propsBus_a.rad.Q_flow;
      for i in 2:nLay loop
        C[i]*der(T[i]) = G[i - 1]*(T[i - 1] - T[i]) - G[i]*(T[i] - T[i + 1]);
      end for;
      C[nLay + 1]*der(T[nLay + 1]) = G[nLay]*(T[nLay] - T[nLay + 1]) - QInt_b + propsBus_b.rad.Q_flow;
      propsBus_a.con.Q_flow = -QInt_a;
      propsBus_a.rad.T = T[1];
      propsBus_a.A = A;
      propsBus_a.epsLw = epsLw;
      propsBus_a.absSol = absSol;
      propsBus_a.tauSol = 0;
      propsBus_a.isFloor = isFloor;
      propsBus_a.QTraDir = 0;
      propsBus_a.QTraDif = 0;
      propsBus_b.con.Q_flow = -QInt_b;
      propsBus_b.rad.T = T[nLay + 1];
      propsBus_b.A = A;
      propsBus_b.epsLw = epsLw;
      propsBus_b.absSol = absSol;
      propsBus_b.tauSol = 0;
      propsBus_b.isFloor = isFloor_b;
      propsBus_b.QTraDir = 0;
      propsBus_b.QTraDif = 0;
    end InternalWall;

    model Window "Glazing with its frame: panes, gas gaps, optics of the glazing (Buildings), sky and wind outside"
      extends PartialSurface;
      parameter Data.Glazing glazing "Glazing, panes from the outside";
      parameter Real frac = 0 "Frame share of the window area";
      parameter Real hFraExt(unit="W/(m2.K)") = 25 "Exterior film coefficient of the frame";
      parameter Real hFraInt(unit="W/(m2.K)") = 7.7 "Interior film coefficient of the frame";
      parameter Real cGla(unit="J/(kg.K)") = 720 "Specific heat capacity of glass when the glazing gives none";
      Real T[glazing.nPan](each unit="K", each start=T_start, each fixed=true) "Pane temperatures, outside first";
      BoundaryConditions.TiltedSurface sol(final azi=azi, final til=til) "Irradiance on the window"
        annotation (Placement(transformation(extent={{-70,60},{-50,80}})));
      Real QExt(unit="W") "Heat from the outer pane to the outside air and the sky";
      Real QInt(unit="W") "Heat from the inner pane to the room air";
      Real QFra(unit="W") "Heat through the frame, from the room air to the outside";
      Real cosInc "Cosine of incidence, 0 when the sun is behind the window";
    protected
      constant Real cosAng[10] = {cos(i*pi/18) for i in 0:9} "Cosine of the tabulated incidence angles";
      parameter Integer nPan = glazing.nPan;
      parameter Real AGla(unit="m2") = A*(1 - frac) "Glass area";
      parameter Real CPan[nPan](each unit="J/K") = {AGla*glazing.x[i]*glazing.d[i]*(if glazing.c[i] > 0 then glazing.c[i]
        else cGla) for i in 1:nPan} "Capacity of each pane";
      parameter Real GGap[nPan - 1](each unit="W/K") = {AGla/(glazing.RPan[i]/2 + glazing.RGap[i] + glazing.RPan[i + 1]/2)
        for i in 1:nPan - 1} "Conductance between two panes";
      parameter Real tauCoe[5] = Utilities.polynomialFit(cosAng, glazing.tau[1:10])
        "Direct transmittance in cos(inc)";
      parameter Real absCoe[nPan, 5] = {Utilities.polynomialFit(cosAng, glazing.absPan[i, 1:10]) for i in 1:nPan}
        "Direct absorptance of each pane in cos(inc)";
      parameter Real GFra(unit="W/K") = A*frac/(1/hFraExt + 1/glazing.UFra + 1/hFraInt) "Frame conductance";
      parameter Real cExtPos = Utilities.convectionCoefficient(pi - til, true);
      parameter Real cExtNeg = Utilities.convectionCoefficient(pi - til, false);
      Real QAbs[nPan](each unit="W") "Absorbed solar radiation of each pane";
    equation
      connect(propsBus_a.wea, sol.weaBus);
      cosInc = Utilities.positivePart(sol.cosInc, 1e-2);
      for i in 1:nPan loop
        QAbs[i] = AGla*(sol.HDir*Utilities.polynomial(absCoe[i, :], cosInc) + sol.HDif*glazing.absPan[i, 11]);
      end for;
      QExt = AGla*(4 + 4*sqrt(propsBus_a.wea.winSpe^2 + 0.01))*(T[1] - propsBus_a.wea.TDryBul)
        + Functions.skyExchange(T[1], AGla, til, glazing.absIR_a, propsBus_a.wea.TDryBul, propsBus_a.wea.TBlaSky);
      QInt = Functions.interiorConvection(T[nPan] - propsBus_a.con.T, AGla, cIntPos, cIntNeg, propsBus_a.hConInt);
      QFra = GFra*(propsBus_a.con.T - propsBus_a.wea.TDryBul);
      if nPan == 1 then
        CPan[1]*der(T[1]) = QAbs[1] - QExt - QInt + propsBus_a.rad.Q_flow;
      else
        CPan[1]*der(T[1]) = QAbs[1] - QExt - GGap[1]*(T[1] - T[2]);
        for i in 2:nPan - 1 loop
          CPan[i]*der(T[i]) = QAbs[i] + GGap[i - 1]*(T[i - 1] - T[i]) - GGap[i]*(T[i] - T[i + 1]);
        end for;
        CPan[nPan]*der(T[nPan]) = QAbs[nPan] + GGap[nPan - 1]*(T[nPan - 1] - T[nPan]) - QInt + propsBus_a.rad.Q_flow;
      end if;
      // the room air loses the heat that leaves through the frame and takes the heat of the inner pane
      propsBus_a.con.Q_flow = QFra - QInt;
      propsBus_a.rad.T = T[nPan];
      propsBus_a.A = AGla;
      propsBus_a.epsLw = glazing.absIR_b;
      propsBus_a.absSol = glazing.absPan[nPan, 11];
      propsBus_a.tauSol = glazing.tau[11];
      propsBus_a.isFloor = 0;
      propsBus_a.QTraDir = AGla*sol.HDir*Utilities.polynomial(tauCoe, cosInc);
      propsBus_a.QTraDif = AGla*sol.HDif*glazing.tau[11];
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Line(points={{-37,-44},{14,-44}}, color={0,0,0}),
          Polygon(points={{14,80},{14,-44},{45,-76},{45,48},{14,80}}, lineColor={0,0,0}, fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Polygon(points={{19,69},{19,-42},{39,-62},{39,48},{19,69}}, lineColor={0,0,0}, fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Line(points={{-51,80},{24,80}}, color={0,0,0}),
          Line(points={{-6,-76},{45,-76}}, color={0,0,0}),
          Polygon(points={{-10,80},{-10,-44},{21,-76},{21,48},{-10,80}}, lineColor={0,0,0},
            fillColor={215,215,215}, fillPattern=FillPattern.Solid),
          Polygon(points={{-5,69},{-5,-42},{15,-62},{15,48},{-5,69}}, lineColor={0,0,0}, fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Polygon(points={{-37,80},{-37,-44},{-6,-76},{-6,48},{-37,80}}, lineColor={0,0,0},
            fillColor={215,215,215}, fillPattern=FillPattern.Solid),
          Polygon(points={{-32,68},{-32,-43},{-12,-63},{-12,47},{-32,68}}, lineColor={0,0,0},
            fillColor={170,213,255}, fillPattern=FillPattern.Solid),
          Line(points={{-17,48},{58,48}}, color={0,0,0}),
          Ellipse(extent={{-113,117},{-82,85}}, lineColor={255,255,0}, fillColor={255,213,170},
            fillPattern=FillPattern.Sphere),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end Window;

    package Functions "Surface heat transfer"
      function interiorConvection "Heat from a room surface to the air: natural convection, or a fixed coefficient"
        input Real dT(unit="K") "Surface minus air temperature";
        input Real A(unit="m2");
        input Real cPos;
        input Real cNeg;
        input Real hFix(unit="W/(m2.K)") "Fixed coefficient, 0 for the natural convection law";
        output Real Q_flow(unit="W");
      algorithm
        Q_flow := if hFix > 0 then hFix*A*dT else Utilities.naturalConvection(dT, A, cPos, cNeg);
      end interiorConvection;

      function skyExchange "Long-wave exchange with sky and ground (Buildings SkyRadiationExchange)"
        input Real T(unit="K") "Surface temperature";
        input Real A(unit="m2");
        input Real til(unit="rad");
        input Real eps "Long-wave emissivity";
        input Real TOut(unit="K");
        input Real TBlaSky(unit="K");
        output Real Q_flow(unit="W") "Heat from the surface";
      protected
        Real vieFacSky = (pi - til)/pi;
        Real TEnv;
      algorithm
        TEnv := (vieFacSky*TBlaSky^4 + (1 - vieFacSky)*TOut^4)^0.25;
        Q_flow := 4*sigma*eps*A*TEnv^3*(T - TEnv);
      end skyExchange;

      function exteriorExchange
        "Heat from an outer surface to the outside: natural and wind-driven convection (Buildings TemperatureWind)
         or a fixed coefficient, and long-wave exchange with sky and ground"
        input Real T(unit="K") "Surface temperature";
        input Real A(unit="m2");
        input Real til(unit="rad");
        input Real azi(unit="rad");
        input Real cPos;
        input Real cNeg;
        input Real eps "Long-wave emissivity";
        input Real hFix(unit="W/(m2.K)") "Fixed convection coefficient, 0 for the wind-dependent law";
        input Real TOut(unit="K");
        input Real TBlaSky(unit="K");
        input Real v(unit="m/s") "Wind speed";
        input Real dir(unit="rad") "Wind direction, 0 from the north";
        input Real R = 1.67 "Surface roughness factor (rough)";
        output Real Q_flow(unit="W");
      protected
        Real W "Wind direction modifier: 1 windward, 0.5 leeward, 0.75 without wind";
        Real hF(unit="W/(m2.K)") "Forced convection coefficient";
      algorithm
        W := 0.75 + (0.5*(0.5 + 0.5*(cos(azi + pi - dir) - cos(1.7453292519943295))
          /sqrt((cos(azi + pi - dir) - cos(1.7453292519943295))^2 + 0.0025)) - 0.25)
          *(0.5 + 0.5*(v - 0.25)/sqrt((v - 0.25)^2 + 0.015625));
        hF := 2.537*W*R*2/A^0.25*(v^2 + 0.0625)^0.25;
        Q_flow := (if hFix > 0 then hFix*A*(T - TOut) else Utilities.naturalConvection(T - TOut, A, cPos, cNeg)
          + A*hF*(T - TOut)) + skyExchange(T, A, til, eps, TOut, TBlaSky);
      end exteriorExchange;
    end Functions;
  end Envelope;

  package Zones "Thermal zones"
    partial model PartialZone
      "Room air, long-wave exchange between its surfaces, distribution of the transmitted solar radiation"
      parameter Integer nSurf(min=1) "Number of surfaces";
      parameter Real AFlo(unit="m2") "Floor area";
      parameter Real hRoo(unit="m") "Height";
      parameter Real V(unit="m3") = AFlo*hRoo "Air volume";
      parameter Real ACH(unit="1/h") = 0 "Air changes per hour by infiltration of outdoor air";
      parameter Real mSenFac = 1 "Factor for the sensible heat capacity of the air";
      parameter Real T_start(unit="K") = 294.15 "Initial temperature";
      parameter Real hConInt(unit="W/(m2.K)") = 0 "Fixed interior convection coefficient, 0 for the natural law";
      parameter Real hConExt(unit="W/(m2.K)") = 0 "Fixed exterior convection coefficient, 0 for the wind law";
      parameter Real T0Rad(unit="K") = 293.15 "Linearization temperature of the long-wave exchange";
      Interfaces.WeatherBusIn weaBus "Weather"
        annotation (Placement(transformation(extent={{-90,90},{-70,110}})));
      Interfaces.ZoneSurfaceBus propsBus[nSurf] "Surfaces"
        annotation (Placement(transformation(extent={{90,-20},{110,20}})));
      Interfaces.HeatPort heaPorAir "Convective heat (emitters)"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Interfaces.HeatPort heaPorRad "Radiant heat (emitters), at the mean surface temperature"
        annotation (Placement(transformation(extent={{-110,-50},{-90,-30}})));
      Interfaces.Gains qGai_flow "Internal gains"
        annotation (Placement(transformation(extent={{-120,30},{-100,50}})));
      Real TAir(unit="K", start=T_start, fixed=true) "Zone air temperature";
      Real TRad(unit="K") "Radiant star temperature";
      Real QVen(unit="W") "Heat brought by the ventilation";
    protected
      Real G[nSurf](each unit="W/K") "Long-wave conductance of each surface to the star";
      Real QDir(unit="W") "Transmitted direct solar radiation";
      Real QDif(unit="W") "Transmitted diffuse solar radiation";
      Real AFloSur(unit="m2") "Area of the floors";
      Real epsTauA(unit="m2") "Absorbing and transmitting area";
      Real kDif[nSurf] "Share of the diffuse radiation absorbed by each surface";
      Real kDir[nSurf] "Share of the direct radiation absorbed by each surface (floors first)";
    equation
      for i in 1:nSurf loop
        connect(weaBus, propsBus[i].wea);
        propsBus[i].hConInt = hConInt;
        propsBus[i].hConExt = hConExt;
        G[i] = 4*sigma*T0Rad^3*propsBus[i].epsLw*propsBus[i].A;
        propsBus[i].con.T = TAir;
      end for;
      // Long-wave exchange through a radiant star: the exact equivalent of Buildings' linearized radiosity exchange
      // with view factors A_j/sum(A); radiant gains reach the surfaces in proportion to epsilon_i*A_i
      TRad = (sum(G[i]*propsBus[i].rad.T for i in 1:nSurf) + heaPorRad.Q_flow + AFlo*qGai_flow.QRad)/sum(G);
      heaPorRad.T = sum(G[i]*propsBus[i].rad.T for i in 1:nSurf)/sum(G);
      // Transmitted solar radiation (Buildings SolarRadiationExchange): direct radiation on the floors first
      QDir = sum(propsBus[i].QTraDir for i in 1:nSurf);
      QDif = sum(propsBus[i].QTraDif for i in 1:nSurf);
      AFloSur = sum(propsBus[i].isFloor*propsBus[i].A for i in 1:nSurf);
      epsTauA = sum((propsBus[i].absSol + propsBus[i].tauSol)*propsBus[i].A for i in 1:nSurf);
      for i in 1:nSurf loop
        kDif[i] = propsBus[i].absSol*propsBus[i].A/epsTauA;
        kDir[i] = noEvent(if AFloSur > 0 then (if propsBus[i].isFloor > 0.5 then propsBus[i].absSol*propsBus[i].A
          /AFloSur else sum(propsBus[j].isFloor*propsBus[j].A*(1 - propsBus[j].absSol - propsBus[j].tauSol)
          for j in 1:nSurf)/AFloSur*propsBus[i].absSol*propsBus[i].A/sum((1 - propsBus[j].isFloor)*(propsBus[j].absSol
          + propsBus[j].tauSol)*propsBus[j].A for j in 1:nSurf)) else kDif[i]);
        propsBus[i].rad.Q_flow = -(G[i]*(TRad - propsBus[i].rad.T) + kDir[i]*QDir + kDif[i]*QDif);
      end for;
      // Air
      mSenFac*rhoAir*cpAir*V*der(TAir) = sum(propsBus[i].con.Q_flow for i in 1:nSurf) + heaPorAir.Q_flow + AFlo*qGai_flow.QCon
        + rhoAir*cpAir*V*ACH/3600*(weaBus.TDryBul - TAir) + QVen;
      heaPorAir.T = TAir;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-80,-80},{80,80}}, lineColor={95,95,95}, fillColor={95,95,95},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{-70,70},{70,-70}}, pattern=LinePattern.None, lineColor={117,148,176},
            fillColor={170,213,255}, fillPattern=FillPattern.Sphere),
          Rectangle(extent={{70,35},{80,-35}}, lineColor={95,95,95}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{73,35},{77,-35}}, lineColor={95,95,95}, fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Text(extent={{-30,6},{-11,-5}}, textColor={0,0,0}, textString="air"),
          Text(extent={{-36,-11},{-11,-25}}, textColor={0,0,0}, textString="radiation"),
          Text(extent={{-52,-62},{-27,-76}}, textColor={0,0,0}, textString="surface"),
          Text(extent={{-150,140},{150,100}}, textColor={0,0,255}, textString="%name")}));
    end PartialZone;

    model Zone "Thermal zone"
      extends PartialZone;
    equation
      QVen = 0;
    end Zone;

    model VentilatedZone "Thermal zone with supply and extract air of a ventilation unit"
      extends PartialZone;
      parameter Integer nPorts_a = 1 "Number of supply air connections";
      parameter Integer nPorts_b = 1 "Number of extract air connections";
      Ventilation.AirInlet ports_a[nPorts_a] "Supply air"
        annotation (Placement(transformation(extent={{-50,-110},{-30,-90}})));
      Ventilation.AirOutlet ports_b[nPorts_b] "Extract air, at the zone air temperature"
        annotation (Placement(transformation(extent={{30,-110},{50,-90}})));
    equation
      // the extract ducts carry the flows of the supply ones: the fan speed passes through the zone
      for i in 1:nPorts_b loop
        ports_b[i].T = TAir;
        ports_b[i].y = sum(ports_a[j].y for j in 1:nPorts_a)/nPorts_a;
      end for;
      QVen = sum(ports_a[i].m_flow*cpAir*(ports_a[i].T - TAir) for i in 1:nPorts_a);
    end VentilatedZone;
  end Zones;

  package Hydronics
    "Water circuits after Buildings.Fluid. Two modes, set by the evaluated parameter pressureDriven of every component:
     set flows (each valve sets the flow of its branch, the production closes the balance: the model of optimal
     control), or pressure-driven flows (closed-loop simulation: a pressure node at the inlet of every component, the
     flow of every outlet from its conductance, the pumps on their curves, as the linearized circuits of Buildings)"
    partial model PartialFlow
      "Water ports as arrays, one element per connection: the inlets mix, every outlet leaves at TOut; with
       pressure-driven flows, the inlets meet in a pressure node of small compliance"
      parameter Integer nPorts_a = 1 "Number of connections of the inlet";
      parameter Integer nPorts_b = 1 "Number of connections of the outlet";
      parameter Real m_flow_small(unit="kg/s") = 1e-6 "Flow below which the inlet takes the mean inlet temperature";
      parameter Boolean pressureDriven = false
        "true: flows from the pressures of the circuit (closed loop); false: flows set by the valves (optimal control)"
        annotation (Evaluate=true);
      parameter Real tauHyd(unit="s") = 5 "Time constant of the pressure of the inlet node (pressure-driven flows)";
      parameter Real GHyd(unit="kg/(s.Pa)") = 1e-4 "Conductance scaling the compliance of the inlet node";
      Interfaces.WaterInlet port_a[nPorts_a] "Inlet"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Interfaces.WaterOutlet port_b[nPorts_b] "Outlet"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Real m_flow(unit="kg/s") "Flow through the component";
      Real TIn(unit="K") "Mixed inlet temperature";
      Real TOut(unit="K") "Outlet temperature";
      Real pIn(unit="Pa", start=0, fixed=pressureDriven) "Pressure of the inlet node";
      Real mTra(unit="kg/s") "Flow carrying the inlet water through (its positive part with pressure-driven flows)";
    equation
      m_flow = sum(port_a[i].m_flow for i in 1:nPorts_a);
      if pressureDriven then
        // flows may reverse while the pressures settle: the inlets mix with the positive part of their flows
        mTra = Utilities.positivePart(m_flow, m_flow_small);
        TIn = (sum(Utilities.positivePart(port_a[i].m_flow, m_flow_small)*port_a[i].T for i in 1:nPorts_a)
          + m_flow_small*sum(port_a[i].T for i in 1:nPorts_a)/nPorts_a)
          /(sum(Utilities.positivePart(port_a[i].m_flow, m_flow_small) for i in 1:nPorts_a) + m_flow_small);
      else
        mTra = m_flow;
        TIn = (sum(port_a[i].m_flow*port_a[i].T for i in 1:nPorts_a) + m_flow_small*sum(port_a[i].T for i in 1:nPorts_a)
          /nPorts_a)/(m_flow + m_flow_small);
      end if;
      for i in 1:nPorts_b loop
        port_b[i].T = TOut;
      end for;
      for i in 1:nPorts_a loop
        port_a[i].p = pIn;
      end for;
      if pressureDriven then
        tauHyd*GHyd*der(pIn) = m_flow + sum(port_b[i].m_flow for i in 1:nPorts_b);
      else
        pIn = 0;
      end if;
    end PartialFlow;

    partial model PartialConserving "Mass is conserved: what enters leaves"
      extends PartialFlow;
    equation
      if not pressureDriven then
        m_flow + sum(port_b[i].m_flow for i in 1:nPorts_b) = 0;
      end if;
    end PartialConserving;

    partial model PartialLink
      "Pressure-driven outlets of a component without resistance in Buildings: a small linear one, dpLink at the
       nominal flow of each outlet"
      extends PartialConserving(final GHyd=nPorts_b*GLink);
      parameter Real m_flow_nominal(unit="kg/s") = 0.15 "Nominal flow";
      parameter Real dpLink(unit="Pa") = 1000 "Pressure drop at the nominal flow (pressure-driven flows)";
      final parameter Real GLink(unit="kg/(s.Pa)") = m_flow_nominal/nPorts_b/dpLink "Conductance of each outlet";
    equation
      if pressureDriven then
        for i in 1:nPorts_b loop
          port_b[i].m_flow = -GLink*(pIn - port_b[i].p);
        end for;
      end if;
    end PartialLink;

    model Pipe "Pass-through (junctions, sensors, pumps without a power model)"
      extends PartialLink;
    equation
      TOut = TIn;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,40},{100,-40}}, lineColor={0,0,0}, fillPattern=FillPattern.HorizontalCylinder,
            fillColor={192,192,192}),
          Rectangle(extent={{-100,22},{100,-24}}, lineColor={0,0,0}, fillPattern=FillPattern.HorizontalCylinder,
            fillColor={0,127,255}),
          Text(extent={{-150,-110},{150,-150}}, textColor={0,0,255}, textString="%name")}));
    end Pipe;

    model TemperatureSensor "Pass-through: T is the water temperature"
      extends PartialLink;
      Interfaces.RealOutput T(unit="K") "Measured temperature"
        annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,110})));
    equation
      TOut = TIn;
      T = TIn;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Line(points={{-100,0},{92,0}}, color={0,128,255}),
          Ellipse(extent={{-20,-58},{20,-20}}, lineColor={0,0,0}, lineThickness=0.5, fillColor={191,0,0},
            fillPattern=FillPattern.Solid),
          Line(points={{-40,60},{-12,60}}),
          Line(points={{-40,30},{-12,30}}),
          Line(points={{-40,0},{-12,0}}),
          Rectangle(extent={{-12,60},{12,-24}}, lineColor={191,0,0}, fillColor={191,0,0},
            fillPattern=FillPattern.Solid),
          Polygon(points={{-12,60},{-12,80},{-10,86},{-6,88},{0,90},{6,88},{10,86},{12,80},{12,60},{-12,60}},
            lineColor={0,0,0}, lineThickness=0.5),
          Text(extent={{102,140},{-18,90}}, textColor={0,0,0}, textString="T"),
          Line(points={{-12,60},{-12,-25}}, thickness=0.5),
          Line(points={{12,60},{12,-24}}, thickness=0.5),
          Line(points={{0,100},{0,50}}, color={0,0,127}),
          Text(extent={{-150,-110},{150,-150}}, textColor={0,0,255}, textString="%name")}));
    end TemperatureSensor;

    model Pump
      "Pump: pass-through with set flows; with pressure-driven flows, the flow at its head from the curve of
       Buildings.Fluid.Movers.Preconfigured.SpeedControlled_y (1.14, 1 and 0.42 times dp_nominal at 0, 1 and 2 times
       the nominal flow), scaled to the speed y by the affinity laws"
      extends PartialConserving(final GHyd=m_flow_nominal/dp_nominal);
      parameter Real dp_nominal(unit="Pa") = 3e4 "Pressure head at the nominal flow";
      parameter Real m_flow_nominal(unit="kg/s") = 1 "Nominal flow";
      parameter Real eta = 0.5 "Overall efficiency";
      Interfaces.RealInput y(unit="1") "Relative speed (pressure-driven flows)"
        annotation (Placement(transformation(extent={{-20,-20},{20,20}}, rotation=270, origin={0,120})));
      Real dp(unit="Pa") "Head";
      Real P(unit="W") "Electrical power";
    protected
      constant Real a = 1.14 "Head at zero flow, relative";
      constant Real b = 0.08 "Quadratic fit of the curve through (0, 1.14), (1, 1), (2, 0.42)";
      constant Real c = -0.22;
      Real x[nPorts_b] "Flow of each outlet relative to y*m_flow_nominal/nPorts_b";
    equation
      TOut = TIn;
      dp = sum(port_b[i].p for i in 1:nPorts_b)/nPorts_b - pIn;
      if pressureDriven then
        for i in 1:nPorts_b loop
          // beyond the shut-off head the flow stays at the top of the curve: the root takes a positive argument
          x[i] = (-b - sqrt(Utilities.positivePart(b^2 - 4*c*(a - (port_b[i].p - pIn)/dp_nominal/(y^2 + 1e-4)), 1e-3)))
            /(2*c);
          port_b[i].m_flow = -y*m_flow_nominal/nPorts_b*x[i];
        end for;
        // the forward flow and head only: while the pressures settle, the pump draws no negative power
        P = Utilities.positivePart(dp, 1)*mTra/rhoWat/eta;
      else
        x = zeros(nPorts_b);
        P = dp_nominal*m_flow/rhoWat/eta;
      end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Line(points={{0,100},{0,50}}, color={0,0,0}),
          Rectangle(extent={{-100,16},{100,-16}}, lineColor={0,0,0}, fillColor={0,127,255},
            fillPattern=FillPattern.HorizontalCylinder),
          Ellipse(extent={{-58,58},{58,-58}}, lineColor={0,0,0}, fillPattern=FillPattern.Sphere,
            fillColor={0,100,199}),
          Polygon(points={{0,50},{0,-50},{54,0},{0,50}}, lineColor={0,0,0}, pattern=LinePattern.None,
            fillPattern=FillPattern.HorizontalCylinder, fillColor={255,255,255}),
          Text(extent={{64,106},{114,92}}, textColor={0,0,127}, textString="P"),
          Text(extent={{-150,-110},{150,-150}}, textColor={0,0,255}, textString="%name")}));
    end Pump;

    model Valve
      "Two-way valve: sets the flow of its branch to y*m_flow_nominal (set flows), or opens to y as
       Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage, linearized (pressure-driven flows)"
      extends PartialConserving(final GHyd=GNom);
      parameter Real m_flow_nominal(unit="kg/s") "Nominal flow";
      parameter Real dpValve_nominal(unit="Pa") = 1e4 "Pressure drop of the open valve at the nominal flow";
      parameter Real dpFixed_nominal(unit="Pa") = 0 "Pressure drop of the pipes in series at the nominal flow";
      parameter Real R = 50 "Rangeability";
      parameter Real l = 1e-4 "Leakage";
      parameter Real delta0 = 0.01 "Linear range of the characteristic near closure";
      final parameter Real GNom(unit="kg/(s.Pa)") = m_flow_nominal/(dpValve_nominal + dpFixed_nominal)
        "Conductance of the open valve and its pipes";
      Interfaces.RealInput y(unit="1") "Flow relative to the nominal flow (set flows), or opening (pressure-driven)"
        annotation (Placement(transformation(extent={{-20,-20},{20,20}}, rotation=270, origin={0,120})));
      Real G(unit="kg/(s.Pa)") "Conductance (pressure-driven flows)";
    equation
      TOut = TIn;
      if pressureDriven then
        G = Utilities.valveConductance(Utilities.equalPercentage(Utilities.smoothLimit(y, 0, 1, 1e-3), R, l, delta0),
          m_flow_nominal, dpValve_nominal, dpFixed_nominal);
        for i in 1:nPorts_b loop
          port_b[i].m_flow = -G/nPorts_b*(pIn - port_b[i].p);
        end for;
      else
        G = 0;
        m_flow = y*m_flow_nominal;
      end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,40},{100,-40}}, lineColor={0,0,0}, fillPattern=FillPattern.HorizontalCylinder,
            fillColor={192,192,192}),
          Rectangle(extent={{-100,22},{100,-24}}, lineColor={0,0,0}, fillPattern=FillPattern.HorizontalCylinder,
            fillColor={0,127,255}),
          Rectangle(extent={{-60,40},{60,-40}}, fillColor={255,255,255}, fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Polygon(points={{0,0},{-76,60},{-76,-60},{0,0}}, lineColor={0,0,0}, fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Polygon(points={{0,0},{76,60},{76,-60},{0,0}}, lineColor={0,0,0}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(points={{0,40},{0,0}}),
          Line(points={{0,100},{0,40}}),
          Text(extent={{-150,-110},{150,-150}}, textColor={0,0,255}, textString="%name")}));
    end Valve;

    model ThreeWayValve
      "Mixing valve: the share y of the outlet flow comes from port_1 (supply), 1 - y from port_3 (set flows), or
       Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear, linearized: port_1 equal-percentage on y,
       port_3 linear on 1 - y (pressure-driven flows)"
      parameter Integer nPorts_1 = 1 "Number of connections of the supply inlet";
      parameter Integer nPorts_2 = 1 "Number of connections of the outlet";
      parameter Integer nPorts_3 = 1 "Number of connections of the bypass inlet";
      parameter Real m_flow_nominal(unit="kg/s") = 1 "Nominal flow";
      parameter Real m_flow_small(unit="kg/s") = 1e-6;
      parameter Boolean pressureDriven = false "true: flows from the pressures of the circuit" annotation (Evaluate=true);
      parameter Real tauHyd(unit="s") = 5 "Time constant of the pressure of the inlet nodes";
      parameter Real dpValve_nominal(unit="Pa") = 6000 "Pressure drop of the open path at the nominal flow";
      parameter Real dpFixed_nominal[2](each unit="Pa") = {0, 0} "Pressure drops in series with port_1 and port_3";
      parameter Real fraK = 0.7 "Kv of the bypass path relative to the supply path";
      parameter Real R = 50 "Rangeability";
      parameter Real l[2] = {0.01, 0.01} "Leakages of the supply and bypass paths";
      parameter Real delta0 = 0.01 "Linear range of the characteristic near closure";
      final parameter Real G1Nom(unit="kg/(s.Pa)") = m_flow_nominal/(dpValve_nominal + dpFixed_nominal[1])
        "Conductance of the open supply path";
      final parameter Real G3Nom(unit="kg/(s.Pa)") = m_flow_nominal/(dpValve_nominal/fraK^2 + dpFixed_nominal[2])
        "Conductance of the open bypass path";
      Interfaces.WaterInlet port_1[nPorts_1] "Supply inlet"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Interfaces.WaterOutlet port_2[nPorts_2] "Mixed outlet"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Interfaces.WaterInlet port_3[nPorts_3] "Bypass inlet"
        annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
      Interfaces.RealInput y(unit="1") "Share of the supply (set flows), or opening of the supply path, 0 to 1"
        annotation (Placement(transformation(extent={{-20,-20},{20,20}}, rotation=270, origin={0,120})));
      Real m_flow(unit="kg/s") "Outlet flow";
      Real m1(unit="kg/s") "Flow of the supply path";
      Real m3(unit="kg/s") "Flow of the bypass path";
      Real T1(unit="K") "Supply temperature";
      Real T3(unit="K") "Bypass temperature";
      Real p1(unit="Pa", start=0, fixed=pressureDriven) "Pressure of the supply inlet node";
      Real p3(unit="Pa", start=0, fixed=pressureDriven) "Pressure of the bypass inlet node";
      Real G1(unit="kg/(s.Pa)") "Conductance of the supply path";
      Real G3(unit="kg/(s.Pa)") "Conductance of the bypass path";
    protected
      Real ys = Utilities.smoothLimit(y, 0, 1, 1e-3);
      Real p2 = sum(port_2[i].p for i in 1:nPorts_2)/nPorts_2 "Pressure of the outlet node";
    equation
      m_flow = -sum(port_2[i].m_flow for i in 1:nPorts_2);
      for i in 1:nPorts_1 loop
        port_1[i].p = p1;
      end for;
      for i in 1:nPorts_3 loop
        port_3[i].p = p3;
      end for;
      if pressureDriven then
        T1 = (sum(Utilities.positivePart(port_1[i].m_flow, m_flow_small)*port_1[i].T for i in 1:nPorts_1)
          + m_flow_small*sum(port_1[i].T for i in 1:nPorts_1)/nPorts_1)
          /(sum(Utilities.positivePart(port_1[i].m_flow, m_flow_small) for i in 1:nPorts_1) + m_flow_small);
        T3 = (sum(Utilities.positivePart(port_3[i].m_flow, m_flow_small)*port_3[i].T for i in 1:nPorts_3)
          + m_flow_small*sum(port_3[i].T for i in 1:nPorts_3)/nPorts_3)
          /(sum(Utilities.positivePart(port_3[i].m_flow, m_flow_small) for i in 1:nPorts_3) + m_flow_small);
        G1 = Utilities.valveConductance(Utilities.equalPercentage(ys, R, l[1], delta0), m_flow_nominal,
          dpValve_nominal, dpFixed_nominal[1]);
        G3 = Utilities.valveConductance(l[2] + (1 - ys)*(1 - l[2]), m_flow_nominal, dpValve_nominal/fraK^2,
          dpFixed_nominal[2]);
        m1 = G1*(p1 - p2);
        m3 = G3*(p3 - p2);
        tauHyd*G1Nom*der(p1) = sum(port_1[i].m_flow for i in 1:nPorts_1) - m1;
        tauHyd*G3Nom*der(p3) = sum(port_3[i].m_flow for i in 1:nPorts_3) - m3;
        for i in 1:nPorts_2 loop
          port_2[i].m_flow = -(m1 + m3)/nPorts_2;
          port_2[i].T = (Utilities.positivePart(m1, 1e-5)*T1 + Utilities.positivePart(m3, 1e-5)*T3)
            /(Utilities.positivePart(m1, 1e-5) + Utilities.positivePart(m3, 1e-5));
        end for;
      else
        G1 = 0;
        G3 = 0;
        p1 = 0;
        p3 = 0;
        T1 = (sum(port_1[i].m_flow*port_1[i].T for i in 1:nPorts_1) + m_flow_small*sum(port_1[i].T for i in 1:nPorts_1)
          /nPorts_1)/(sum(port_1[i].m_flow for i in 1:nPorts_1) + m_flow_small);
        T3 = (sum(port_3[i].m_flow*port_3[i].T for i in 1:nPorts_3) + m_flow_small*sum(port_3[i].T for i in 1:nPorts_3)
          /nPorts_3)/(sum(port_3[i].m_flow for i in 1:nPorts_3) + m_flow_small);
        m1 = sum(port_1[i].m_flow for i in 1:nPorts_1);
        m3 = sum(port_3[i].m_flow for i in 1:nPorts_3);
        m1 + m3 = m_flow;
        m1 = y*m_flow;
        for i in 1:nPorts_2 loop
          port_2[i].T = y*T1 + (1 - y)*T3;
        end for;
      end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,40},{100,-40}}, lineColor={0,0,0}, fillPattern=FillPattern.HorizontalCylinder,
            fillColor={192,192,192}),
          Rectangle(extent={{-100,22},{100,-22}}, lineColor={0,0,0}, fillPattern=FillPattern.HorizontalCylinder,
            fillColor={0,127,255}),
          Rectangle(extent={{-60,40},{60,-40}}, fillColor={255,255,255}, fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Polygon(points={{0,0},{-76,60},{-76,-60},{0,0}}, lineColor={0,0,0}, fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Polygon(points={{0,0},{76,60},{76,-60},{0,0}}, lineColor={0,0,0}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{-40,-56},{40,-100}}, lineColor={0,0,0}, fillPattern=FillPattern.VerticalCylinder,
            fillColor={192,192,192}),
          Rectangle(extent={{-22,-56},{22,-100}}, lineColor={0,0,0}, fillPattern=FillPattern.VerticalCylinder,
            fillColor={0,127,255}),
          Polygon(points={{0,0},{60,-76},{-60,-76},{0,0}}, lineColor={0,0,0}, fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Line(points={{0,40},{0,0}}),
          Line(points={{0,100},{0,40}}),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end ThreeWayValve;

    model Junction
      "Split valve: its inlets (port_1) mix, its outlets (port_2, port_3) leave at the mixed temperature; with
       pressure-driven flows the resistance of Buildings' Junction (dp_nominal[1] at m_flow_nominal[1], linearized) is
       placed on each outlet"
      parameter Integer nPorts_1 = 1 "Number of connections of the inlet";
      parameter Integer nPorts_2 = 1 "Number of connections of the outlet";
      parameter Integer nPorts_3 = 1 "Number of connections of the second outlet";
      parameter Real m_flow_small(unit="kg/s") = 1e-6;
      parameter Boolean pressureDriven = false "true: flows from the pressures of the circuit" annotation (Evaluate=true);
      parameter Real tauHyd(unit="s") = 5 "Time constant of the pressure of the inlet node";
      parameter Real dp_nominal[3](each unit="Pa") = {5000, -1, -1} "Pressure drops of Buildings' Junction";
      parameter Real m_flow_nominal[3](each unit="kg/s") = 0.15*{1, -1, -1} "Nominal flows of Buildings' Junction";
      parameter Real dpLink(unit="Pa") = 1000 "Pressure drop at the nominal flow when dp_nominal[1] is not positive";
      final parameter Real G(unit="kg/(s.Pa)") = abs(m_flow_nominal[1])/max(dp_nominal[1], dpLink)
        "Conductance of each outlet (dp_nominal[1], at least dpLink)";
      Interfaces.WaterInlet port_1[nPorts_1] "Inlets"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Interfaces.WaterOutlet port_2[nPorts_2] "Outlet"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Interfaces.WaterOutlet port_3[nPorts_3] "Second outlet (bypass)"
        annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
      Real T(unit="K") "Mixed temperature";
      Real pIn(unit="Pa", start=0, fixed=pressureDriven) "Pressure of the inlet node";
    equation
      for i in 1:nPorts_1 loop
        port_1[i].p = pIn;
      end for;
      T = if pressureDriven then (sum(Utilities.positivePart(port_1[i].m_flow, m_flow_small)*port_1[i].T
        for i in 1:nPorts_1) + m_flow_small*sum(port_1[i].T for i in 1:nPorts_1)/nPorts_1)
        /(sum(Utilities.positivePart(port_1[i].m_flow, m_flow_small) for i in 1:nPorts_1) + m_flow_small)
        else (sum(port_1[i].m_flow*port_1[i].T for i in 1:nPorts_1) + m_flow_small*sum(port_1[i].T for i in 1:nPorts_1)
        /nPorts_1)/(sum(port_1[i].m_flow for i in 1:nPorts_1) + m_flow_small);
      for i in 1:nPorts_2 loop
        port_2[i].T = T;
      end for;
      for i in 1:nPorts_3 loop
        port_3[i].T = T;
      end for;
      if pressureDriven then
        tauHyd*G*(nPorts_2 + nPorts_3)*der(pIn) = sum(port_1[i].m_flow for i in 1:nPorts_1)
          + sum(port_2[i].m_flow for i in 1:nPorts_2) + sum(port_3[i].m_flow for i in 1:nPorts_3);
        for i in 1:nPorts_2 loop
          port_2[i].m_flow = -G*(pIn - port_2[i].p);
        end for;
        for i in 1:nPorts_3 loop
          port_3[i].m_flow = -G*(pIn - port_3[i].p);
        end for;
      else
        pIn = 0;
        sum(port_1[i].m_flow for i in 1:nPorts_1) + sum(port_2[i].m_flow for i in 1:nPorts_2)
          + sum(port_3[i].m_flow for i in 1:nPorts_3) = 0;
      end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Polygon(points={{-100,-46},{-32,-40},{-32,-100},{30,-100},{30,-36},{100,-30},{100,38},{-100,52},{-100,-46}},
            lineColor={0,0,0}, fillColor={175,175,175}, fillPattern=FillPattern.Solid),
          Polygon(points={{-100,-34},{-18,-28},{-18,-100},{18,-100},{18,-26},{100,-20},{100,22},{-100,38},{-100,-34}},
            lineColor={0,0,0}, fillPattern=FillPattern.HorizontalCylinder, fillColor={0,128,255}),
          Text(extent={{-151,142},{149,102}}, textColor={0,0,255}, textString="%name")}));
    end Junction;

    model Junction2 "Split valve without bypass: its inlets (port_1) mix, its outlets (port_2) leave at the mixed temperature"
      parameter Integer nPorts_1 = 1 "Number of connections of the inlet";
      parameter Integer nPorts_2 = 1 "Number of connections of the outlet";
      parameter Real m_flow_small(unit="kg/s") = 1e-6;
      parameter Boolean pressureDriven = false "true: flows from the pressures of the circuit" annotation (Evaluate=true);
      parameter Real tauHyd(unit="s") = 5 "Time constant of the pressure of the inlet node";
      parameter Real dp_nominal[3](each unit="Pa") = {5000, -1, -1} "Pressure drops of Buildings' Junction";
      parameter Real m_flow_nominal[3](each unit="kg/s") = 0.15*{1, -1, -1} "Nominal flows of Buildings' Junction";
      parameter Real dpLink(unit="Pa") = 1000 "Pressure drop at the nominal flow when dp_nominal[1] is not positive";
      final parameter Real G(unit="kg/(s.Pa)") = abs(m_flow_nominal[1])/max(dp_nominal[1], dpLink)
        "Conductance of each outlet (dp_nominal[1], at least dpLink)";
      Interfaces.WaterInlet port_1[nPorts_1] "Inlets"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Interfaces.WaterOutlet port_2[nPorts_2] "Outlets"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Real T(unit="K") "Mixed temperature";
      Real pIn(unit="Pa", start=0, fixed=pressureDriven) "Pressure of the inlet node";
    equation
      for i in 1:nPorts_1 loop
        port_1[i].p = pIn;
      end for;
      T = if pressureDriven then (sum(Utilities.positivePart(port_1[i].m_flow, m_flow_small)*port_1[i].T
        for i in 1:nPorts_1) + m_flow_small*sum(port_1[i].T for i in 1:nPorts_1)/nPorts_1)
        /(sum(Utilities.positivePart(port_1[i].m_flow, m_flow_small) for i in 1:nPorts_1) + m_flow_small)
        else (sum(port_1[i].m_flow*port_1[i].T for i in 1:nPorts_1) + m_flow_small*sum(port_1[i].T for i in 1:nPorts_1)
        /nPorts_1)/(sum(port_1[i].m_flow for i in 1:nPorts_1) + m_flow_small);
      for i in 1:nPorts_2 loop
        port_2[i].T = T;
      end for;
      if pressureDriven then
        tauHyd*G*nPorts_2*der(pIn) = sum(port_1[i].m_flow for i in 1:nPorts_1) + sum(port_2[i].m_flow for i in 1:nPorts_2);
        for i in 1:nPorts_2 loop
          port_2[i].m_flow = -G*(pIn - port_2[i].p);
        end for;
      else
        pIn = 0;
        sum(port_1[i].m_flow for i in 1:nPorts_1) + sum(port_2[i].m_flow for i in 1:nPorts_2) = 0;
      end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Polygon(points={{-100,-40},{100,-40},{100,40},{-100,40},{-100,-40}}, lineColor={0,0,0},
            fillColor={175,175,175}, fillPattern=FillPattern.Solid),
          Polygon(points={{-100,-22},{100,-22},{100,22},{-100,22},{-100,-22}}, lineColor={0,0,0},
            fillPattern=FillPattern.HorizontalCylinder, fillColor={0,128,255}),
          Ellipse(extent={{-12,12},{12,-12}}, lineColor={0,0,0}, fillColor={0,0,0}, fillPattern=FillPattern.Solid),
          Text(extent={{-151,142},{149,102}}, textColor={0,0,255}, textString="%name")}));
    end Junction2;

    function radiatorConductance
      "UAEle of Buildings RadiatorEN442_2: the conductance of each element giving the nominal power (bisections)"
      input Real Q_flow_nominal(unit="W");
      input Real T_a_nominal(unit="K");
      input Real T_b_nominal(unit="K");
      input Real TAir_nominal(unit="K");
      input Real TRad_nominal(unit="K");
      input Real fraRad;
      input Real n;
      input Integer nEle;
      output Real UAEle(unit="W/K");
    protected
      Real m_flow;
      Real UALow;
      Real UAHigh;
      Real TUp;
      Real TLow;
      Real THigh;
      Real TWat;
      Real Q;
      Real QEle;
      Real residual;
    algorithm
      m_flow := Q_flow_nominal/cpWat/(T_a_nominal - T_b_nominal);
      UAEle := Q_flow_nominal/((T_a_nominal + T_b_nominal)/2 - ((1 - fraRad)*TAir_nominal + fraRad*TRad_nominal))^n/nEle;
      UALow := UAEle/10;
      UAHigh := UAEle*10;
      for iteration in 1:80 loop
        UAEle := (UALow + UAHigh)/2;
        // power of the radiator: the water of each element is at the temperature it leaves with
        TUp := T_a_nominal;
        Q := 0;
        for i in 1:nEle loop
          TLow := min(TAir_nominal, TRad_nominal);
          THigh := TUp;
          for k in 1:80 loop
            TWat := (TLow + THigh)/2;
            QEle := UAEle*(fraRad*max(TWat - TRad_nominal, 0)^n + (1 - fraRad)*max(TWat - TAir_nominal, 0)^n);
            residual := TWat - TUp + QEle/m_flow/cpWat;
            if residual > 0 then
              THigh := TWat;
            else
              TLow := TWat;
            end if;
          end for;
          Q := Q + UAEle*(fraRad*max(TWat - TRad_nominal, 0)^n + (1 - fraRad)*max(TWat - TAir_nominal, 0)^n);
          TUp := TWat;
        end for;
        if Q > Q_flow_nominal then
          UAHigh := UAEle;
        else
          UALow := UAEle;
        end if;
      end for;
    end radiatorConductance;

    model Radiator "EN 442 radiator (Buildings RadiatorEN442_2): nEle water and metal elements in series"
      extends PartialConserving(final GHyd=m_flow_nominal/dp_nominal);
      parameter Real dp_nominal(unit="Pa") = 2000 "Pressure drop at the nominal flow, linearized (pressure-driven flows)";
      parameter Real Q_flow_nominal(unit="W") "Nominal heating power"
        annotation (Evaluate=true);
      parameter Real T_a_nominal(unit="K") "Water inlet temperature at nominal conditions"
        annotation (Evaluate=true);
      parameter Real T_b_nominal(unit="K") "Water outlet temperature at nominal conditions"
        annotation (Evaluate=true);
      parameter Real TAir_nominal(unit="K") = 293.15 "Air temperature at nominal conditions"
        annotation (Evaluate=true);
      parameter Real TRad_nominal(unit="K") = TAir_nominal "Radiative temperature at nominal conditions"
        annotation (Evaluate=true);
      parameter Real n = 1.24 "Exponent of the EN 442 power law"
        annotation (Evaluate=true);
      parameter Real fraRad = 0.35 "Radiant share of the heat output"
        annotation (Evaluate=true);
      parameter Integer nEle = 5 "Number of elements"
        annotation (Evaluate=true);
      parameter Real VWat(unit="m3") = 5.8e-6*abs(Q_flow_nominal) "Water volume";
      parameter Real mDry(unit="kg") = 0.0263*abs(Q_flow_nominal) "Dry mass lumped to the water";
      parameter Real T_start(unit="K") = 293.15 "Initial water temperature";
      parameter Real dT0(unit="K") = 0.05 "Regularisation of the power law around a zero temperature difference";
      parameter Real UAEle(unit="W/K") = radiatorConductance(Q_flow_nominal, T_a_nominal, T_b_nominal, TAir_nominal,
        TRad_nominal, fraRad, n, nEle) "Conductance of each element";
      parameter Real C(unit="J/K") = rhoWat*VWat*cpWat + 500*mDry "Heat capacity of the water and of the dry mass";
      final parameter Real m_flow_nominal(unit="kg/s") = abs(Q_flow_nominal/cpWat/(T_a_nominal - T_b_nominal))
        "Nominal flow";
      Real T[nEle](each unit="K", each start=T_start, each fixed=true) "Water temperature of each element";
      Interfaces.HeatPort heatPortCon "Convective heat to the room air"
        annotation (Placement(transformation(extent={{-30,62},{-10,82}})));
      Interfaces.HeatPort heatPortRad "Radiative heat to the room surfaces"
        annotation (Placement(transformation(extent={{10,62},{30,82}})));
      Real Q_flow(unit="W") "Heat output";
    protected
      Real QCon[nEle](each unit="W") "Convective heat output of each element";
      Real QRad[nEle](each unit="W") "Radiative heat output of each element";
      Real TUp[nEle](each unit="K") "Temperature of the water entering each element";
    equation
      for i in 1:nEle loop
        TUp[i] = if i == 1 then TIn else T[max(i - 1, 1)];
        QCon[i] = (1 - fraRad)*UAEle*(T[i] - heatPortCon.T)*((T[i] - heatPortCon.T)^2 + dT0^2)^((n - 1)/2);
        QRad[i] = fraRad*UAEle*(T[i] - heatPortRad.T)*((T[i] - heatPortRad.T)^2 + dT0^2)^((n - 1)/2);
        C/nEle*der(T[i]) = mTra*cpWat*(TUp[i] - T[i]) - QCon[i] - QRad[i];
      end for;
      TOut = T[nEle];
      if pressureDriven then
        for i in 1:nPorts_b loop
          port_b[i].m_flow = -GHyd/nPorts_b*(pIn - port_b[i].p);
        end for;
      end if;
      Q_flow = sum(QCon) + sum(QRad);
      heatPortCon.Q_flow = -sum(QCon);
      heatPortRad.Q_flow = -sum(QRad);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-95,6},{106,-4}}, lineColor={0,0,255}, pattern=LinePattern.None, fillColor={0,0,255},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{-102,-4},{-2,6}}, lineColor={0,0,255}, pattern=LinePattern.None, fillColor={255,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{-80,60},{80,-60}}, lineColor={0,0,0}, fillColor={95,95,95},
            fillPattern=FillPattern.Solid),
          Line(points={{-66,30},{66,30}}),
          Line(points={{-66,2},{66,2}}),
          Line(points={{-66,-30},{66,-30}}),
          Line(points={{-66,60},{-66,-60}}),
          Line(points={{66,60},{66,-60}}),
          Text(extent={{-150,-110},{150,-150}}, textColor={0,0,255}, textString="%name")}));
    end Radiator;

    model Boiler
      "Boiler (Buildings BoilerPolynomial): water and metal heated by the part load ratio y; with set flows, the
       reference of its circuit (no mass balance)"
      extends PartialFlow(final GHyd=m_flow_nominal/dp_nominal);
      parameter Real m_flow_nominal(unit="kg/s") = Q_flow_nominal/cpWat/20 "Nominal flow (pressure-driven flows)";
      parameter Real dp_nominal(unit="Pa") = 5000 "Pressure drop at the nominal flow (pressure-driven flows)";
      parameter Real deltaM = 0.1 "Regularization of the quadratic flow law below deltaM*m_flow_nominal";
      parameter Real Q_flow_nominal(unit="W") "Nominal heating power";
      parameter Integer effCur = Types.EfficiencyCurves.Constant "Efficiency curve";
      parameter Real a[:] = {0.9} "Coefficients of the efficiency curve (Buildings)";
      parameter Real T_nominal(unit="K") = 353.15 "Temperature used to compute the nominal efficiency";
      parameter Real VWat(unit="m3") = 1.5e-6*Q_flow_nominal "Water volume";
      parameter Real mDry(unit="kg") = 1.5e-3*Q_flow_nominal "Dry mass";
      parameter Real UA(unit="W/K") = 2*1e-4*Q_flow_nominal
        "Heat loss conductance to the boiler room (Buildings 13 BoilerPolynomial counts its UA loss twice)";
      parameter Real TAmb(unit="K") = 288.15 "Temperature of the boiler room";
      parameter Real T_start(unit="K") = 293.15 "Initial water temperature";
      parameter Real C(unit="J/K") = rhoWat*VWat*cpWat + 500*mDry "Heat capacity of the water and of the dry mass";
      Interfaces.RealInput y(unit="1") "Part load ratio, 0 to 1"
        annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
      Real T(unit="K", start=T_start, fixed=true) "Water temperature";
      Real EFue(unit="kWh", start=0, fixed=true) "Fuel meter";
      Real QFue(unit="W") "Fuel power";
      Real QWat(unit="W") "Heat to the water";
      Real eta "Efficiency";
    protected
      parameter Real aPad[6] = cat(1, a, zeros(6 - size(a, 1))) "Coefficients padded with zeros";
      parameter Real eta_nominal = if effCur == Types.EfficiencyCurves.QuadraticLinear then aPad[1] + aPad[2] + aPad[3]
        + (aPad[4] + aPad[5] + aPad[6])*T_nominal else sum(aPad) "Efficiency at nominal conditions";
    equation
      // the reference of its circuit: no mass balance (the circuit closes it), the circuit flows set by its valves
      QFue = y*Q_flow_nominal/eta_nominal;
      eta = if effCur == Types.EfficiencyCurves.QuadraticLinear then aPad[1] + aPad[2]*y + aPad[3]*y^2
        + (aPad[4] + aPad[5]*y + aPad[6]*y^2)*T else aPad[1] + aPad[2]*y + aPad[3]*y^2 + aPad[4]*y^3 + aPad[5]*y^4
        + aPad[6]*y^5;
      QWat = eta*QFue + UA*(TAmb - T);
      C*der(T) = mTra*cpWat*(TIn - T) + QWat;
      der(EFue) = QFue/3.6e6;
      TOut = T;
      if pressureDriven then
        for i in 1:nPorts_b loop
          port_b[i].m_flow = -Utilities.quadraticFlow(pIn - port_b[i].p, m_flow_nominal/nPorts_b, dp_nominal, deltaM);
        end for;
      end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-70,60},{70,-60}}, lineColor={0,0,255}, pattern=LinePattern.None,
            fillColor={95,95,95}, fillPattern=FillPattern.Solid),
          Rectangle(extent={{-101,5},{100,-4}}, lineColor={0,0,255}, pattern=LinePattern.None, fillColor={0,0,255},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{0,-4},{100,5}}, lineColor={0,0,255}, pattern=LinePattern.None, fillColor={255,0,0},
            fillPattern=FillPattern.Solid),
          Polygon(points={{0,-34},{-12,-52},{14,-52},{0,-34}}, pattern=LinePattern.None, fillColor={255,255,0},
            fillPattern=FillPattern.Solid, lineColor={0,0,0}),
          Line(points={{-100,80},{-80,80},{-80,-44},{-6,-44}}),
          Text(extent={{-38,146},{-158,96}}, textColor={0,0,0}, textString="y"),
          Text(extent={{-150,-110},{150,-150}}, textColor={0,0,255}, textString="%name")}));
    end Boiler;

    package HeatPumps "Performance data of heat pumps (Buildings.Fluid.HeatPumps.ModularReversible.Data.TableData2D)"
      package Vitocal251A08 "Viessmann Vitocal 251-A08, EN 14511"
        constant Real TSou[9](each unit="K") = {253.15, 258.15, 266.15, 275.15, 280.15, 283.15, 293.15, 303.15, 308.15}
          "Source temperatures (columns)";
        constant Real TCon[5](each unit="K") = {308.15, 318.15, 328.15, 338.15, 343.15} "Condenser outlet (rows)";
        constant Real QHea[5, 9](each unit="W") = {{4490, 5170, 6470, 6790, 8000, 10210, 12330, 12310, 13090},
          {4230, 4900, 6260, 6780, 8370, 9970, 11520, 13040, 12640}, {3780, 4710, 6030, 6830, 8380, 9940, 11500, 13070,
          13110}, {0, 3170, 4610, 6320, 8140, 9550, 11290, 12100, 12180}, {0, 0, 3830, 5560, 7600, 8700, 11290, 12500,
          12590}} "Heating capacity";
        constant Real PEle[5, 9](each unit="W") = {{2128, 2238, 2396, 1835, 1633, 2006, 1720, 1465, 1558},
          {2299, 2402, 2641, 2173, 2244, 2462, 2110, 1734, 1584}, {2305, 2631, 2899, 2493, 2669, 2915, 2556, 2164, 2081},
          {0, 2032, 2478, 2760, 3095, 3375, 3085, 2521, 2511}, {0, 0, 2201, 2699, 3193, 3398, 3330, 2822, 2817}}
          "Electrical power";
        constant Real mCon_flow_nominal(unit="kg/s") = 5600/5/4184 "Nominal condenser flow of the record";

        function interpolate "Linear interpolation in the table, as CombiTable2Ds (rows: condenser, columns: source)"
          input Real TConOut(unit="K");
          input Real TSource(unit="K");
          input Boolean heating "true: heating capacity, false: electrical power";
          output Real y;
        protected
          Real row[5] "Values of each row at the source temperature";
          Real w;
        algorithm
          for r in 1:5 loop
            row[r] := if heating then QHea[r, 1] else PEle[r, 1];
            for j in 1:8 loop
              if TSource > TSou[j] then
                w := min((TSource - TSou[j])/(TSou[j + 1] - TSou[j]), 1);
                row[r] := if heating then QHea[r, j] + w*(QHea[r, j + 1] - QHea[r, j]) else PEle[r, j]
                  + w*(PEle[r, j + 1] - PEle[r, j]);
              end if;
            end for;
          end for;
          y := row[1];
          for r in 1:4 loop
            if TConOut > TCon[r] then
              w := min((TConOut - TCon[r])/(TCon[r + 1] - TCon[r]), 1);
              y := row[r] + w*(row[r + 1] - row[r]);
            end if;
          end for;
        end interpolate;

        function smoothTable
          "Quadratic in the condenser temperature through the rows at 35, 45 and 55 degC, linear between the source
           columns with the kinks smoothed over dTSou: a differentiable version of the table"
          input Real TConOut(unit="K");
          input Real TSource(unit="K");
          input Boolean heating "true: heating capacity, false: electrical power";
          input Real dTSou(unit="K") = 1 "Smoothing of the kinks of the interpolation in the source temperature";
          output Real y;
        protected
          Real v[9] "Value at each source temperature";
          Real L[3] "Lagrange basis in the condenser temperature";
        algorithm
          L[1] := (TConOut - TCon[2])*(TConOut - TCon[3])/((TCon[1] - TCon[2])*(TCon[1] - TCon[3]));
          L[2] := (TConOut - TCon[1])*(TConOut - TCon[3])/((TCon[2] - TCon[1])*(TCon[2] - TCon[3]));
          L[3] := (TConOut - TCon[1])*(TConOut - TCon[2])/((TCon[3] - TCon[1])*(TCon[3] - TCon[2]));
          for j in 1:9 loop
            v[j] := if heating then L[1]*QHea[1, j] + L[2]*QHea[2, j] + L[3]*QHea[3, j] else L[1]*PEle[1, j]
              + L[2]*PEle[2, j] + L[3]*PEle[3, j];
          end for;
          y := v[1] + (v[2] - v[1])/(TSou[2] - TSou[1])*(TSource - TSou[1]);
          for i in 2:8 loop
            y := y + ((v[i + 1] - v[i])/(TSou[i + 1] - TSou[i]) - (v[i] - v[i - 1])/(TSou[i] - TSou[i - 1]))
              *Utilities.positivePart(TSource - TSou[i], dTSou);
          end for;
        end smoothTable;
      end Vitocal251A08;
    end HeatPumps;

    model HeatPump
      "Air-to-water heat pump (Buildings ModularReversible AirToWaterTableData2D, Vitocal 251-A08 scaled to its
       capacity) charging a mixed buffer; with set flows, the reference of its circuit (no mass balance)"
      extends PartialFlow(final GHyd=GLink);
      parameter Real m_flow_nominal(unit="kg/s") = Q_flow_nominal/cpWat/5 "Nominal flow of the heating circuit";
      parameter Real dpLink(unit="Pa") = 1000
        "Pressure drop at the nominal flow (pressure-driven flows; the heating side of Buildings has none)";
      final parameter Real GLink(unit="kg/(s.Pa)") = m_flow_nominal/dpLink;
      parameter Real Q_flow_nominal(unit="W") "Heating capacity at the design outdoor and supply temperatures"
        annotation (Evaluate=true);
      parameter Real TSet(unit="K") = 318.15 "Supply temperature at design conditions"
        annotation (Evaluate=true);
      parameter Real TOutDes(unit="K") = 265.15 "Design outdoor temperature"
        annotation (Evaluate=true);
      parameter Boolean useStorageTank = false "Buffer tank; a hydraulic separator otherwise";
      parameter Real VTan(unit="m3") = 0.1 "Volume of the buffer tank";
      parameter Real hTan(unit="m") = 0.8 "Height of the buffer tank";
      parameter Real dIns(unit="m") = 0.1 "Insulation thickness of the buffer tank";
      parameter Real kIns(unit="W/(m.K)") = 0.04 "Insulation conductivity";
      parameter Real TAmb(unit="K") = 288.15 "Temperature around the buffer";
      parameter Real tauCon(unit="s") = 30 "Time constant of the condenser";
      parameter Real dTSou(unit="K") = 1 "Smoothing of the table interpolation in the source temperature";
      parameter Real scaFac = Q_flow_nominal/HeatPumps.Vitocal251A08.interpolate(TSet, TOutDes, true)
        "Scaling of the table data to the capacity";
      parameter Real mCon_flow(unit="kg/s") = HeatPumps.Vitocal251A08.mCon_flow_nominal*scaFac "Condenser flow";
      parameter Real dTCon(unit="K") = Q_flow_nominal/cpWat/mCon_flow "Condenser outlet above the buffer at full speed";
      parameter Real VWat(unit="m3") = if useStorageTank then VTan else 1.5e-5*Q_flow_nominal "Water volume";
      parameter Real UA(unit="W/K") = if useStorageTank then kIns/dIns*(2*pi*sqrt(VTan/(pi*hTan))*hTan
        + 2*pi*VTan/(pi*hTan)) else 0 "Heat loss conductance of the buffer";
      parameter Real C(unit="J/K") = (rhoWat*VWat + mCon_flow*tauCon)*cpWat "Heat capacity of the buffer and condenser";
      parameter Real T_start(unit="K") = TSet "Initial buffer temperature";
      Interfaces.RealInput y(unit="1") "Compressor speed, 0 to 1"
        annotation (Placement(transformation(extent={{-140,0},{-100,40}})));
      Interfaces.WeatherBusIn weaBus "Weather: the source is the outdoor air"
        annotation (Placement(transformation(extent={{-110,-90},{-90,-70}})));
      Real T(unit="K", start=T_start, fixed=true) "Buffer temperature";
      Real EEle(unit="kWh", start=0, fixed=true) "Electricity meter";
      Real QCon(unit="W") "Heat to the water";
      Real PEle(unit="W") "Electrical power of the compressor";
      Real COP(unit="1") "Coefficient of performance QCon/PEle at the current condenser and source temperatures";
      Real TConOut(unit="K") "Condenser outlet temperature";
    equation
      TConOut = T + dTCon*y;
      QCon = y*scaFac*HeatPumps.Vitocal251A08.smoothTable(TConOut, weaBus.TDryBul, true, dTSou);
      PEle = y*scaFac*HeatPumps.Vitocal251A08.smoothTable(TConOut, weaBus.TDryBul, false, dTSou);
      // QCon and PEle both scale with y: their ratio is defined at every speed, also when the compressor is off
      COP = HeatPumps.Vitocal251A08.smoothTable(TConOut, weaBus.TDryBul, true, dTSou)
        /HeatPumps.Vitocal251A08.smoothTable(TConOut, weaBus.TDryBul, false, dTSou);
      C*der(T) = mTra*cpWat*(TIn - T) + QCon + UA*(TAmb - T);
      der(EEle) = PEle/3.6e6;
      TOut = T;
      if pressureDriven then
        for i in 1:nPorts_b loop
          port_b[i].m_flow = -GLink/nPorts_b*(pIn - port_b[i].p);
        end for;
      end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-102,100},{100,-100}}, lineColor={0,0,0}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{-16,60},{16,-60}}, fillColor={170,213,255}, fillPattern=FillPattern.Solid,
            lineColor={0,0,0}, origin={0,-64}, rotation=90),
          Rectangle(extent={{-16,60},{16,-60}}, fillColor={238,46,47}, fillPattern=FillPattern.Solid,
            lineColor={0,0,0}, origin={0,60}, rotation=90),
          Line(points={{-9,40},{9,40},{-5,-2},{9,-40},{-9,-40}}, color={0,0,0}, origin={-3,-60}, rotation=-90),
          Line(points={{9,40},{-9,40},{5,-2},{-9,-40},{9,-40}}, color={0,0,0}, origin={-5,56}, rotation=-90),
          Rectangle(extent={{-60,42},{60,-46}}, fillColor={0,0,0}, fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Line(points={{-42,72},{34,72}}, color={0,0,0}, arrow={Arrow.None,Arrow.Filled}, thickness=0.5),
          Line(points={{-38,0},{38,0}}, color={0,0,0}, arrow={Arrow.None,Arrow.Filled}, thickness=0.5,
            origin={0,-74}, rotation=180),
          Text(extent={{-96,28},{-74,10}}, textColor={0,0,127}, textString="y"),
          Rectangle(extent={{34,42},{38,-46}}, lineColor={0,0,0}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Ellipse(extent={{14,22},{58,-20}}, lineColor={0,0,0}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(points={{36,22},{18,-10},{54,-10},{36,22}}, lineColor={0,0,0}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{-46,42},{-42,-46}}, lineColor={0,0,0}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(points={{-44,2},{-54,-10},{-34,-10},{-44,2}}, lineColor={0,0,0}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(points={{-44,2},{-54,12},{-34,12},{-44,2}}, lineColor={0,0,0}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end HeatPump;
  end Hydronics;

  package Ventilation "Mechanical ventilation (after trano's Buildings system D)"
    connector AirInlet "Air port, inlet: the temperature of the air crossing it and the speed of its fan"
      flow Real m_flow(unit="kg/s") "Mass flow rate into the component";
      Real T(unit="K") "Temperature of the air crossing the port";
      Interfaces.RealInput y(unit="1") "Relative speed of the fan moving the air";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Ellipse(extent={{-100,100},{100,-100}}, lineColor={0,127,255}, fillColor={0,127,255},
            fillPattern=FillPattern.Solid)}));
    end AirInlet;

    connector AirOutlet "Air port, outlet: the temperature of the air crossing it and the speed of its fan"
      flow Real m_flow(unit="kg/s") "Mass flow rate into the component";
      Real T(unit="K") "Temperature of the air crossing the port";
      Interfaces.RealOutput y(unit="1") "Relative speed of the fan moving the air";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Ellipse(extent={{-100,100},{100,-100}}, lineColor={0,127,255}, fillColor={0,127,255},
            fillPattern=FillPattern.Solid),
          Ellipse(extent={{-80,80},{80,-80}}, lineColor={0,127,255}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
    end AirOutlet;

    model Duct "Duct carrying y*m_flow_nominal: y is the relative speed of the fan of its unit"
      parameter Integer nPorts_a = 1 "Number of connections of the inlet";
      parameter Integer nPorts_b = 1 "Number of connections of the outlet";
      parameter Real m_flow_nominal(unit="kg/s") "Nominal flow";
      AirInlet port_a[nPorts_a]
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      AirOutlet port_b[nPorts_b]
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Real m_flow(unit="kg/s") "Flow through the duct";
    equation
      m_flow = sum(port_a[i].m_flow for i in 1:nPorts_a);
      m_flow = port_a[1].y*m_flow_nominal;
      m_flow + sum(port_b[i].m_flow for i in 1:nPorts_b) = 0;
      for i in 1:nPorts_b loop
        port_b[i].T = port_a[1].T;
        port_b[i].y = port_a[1].y;
      end for;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,40},{100,-40}}, lineColor={0,0,0}, fillPattern=FillPattern.HorizontalCylinder,
            fillColor={192,192,192}),
          Rectangle(extent={{-100,22},{100,-24}}, lineColor={0,0,0}, fillPattern=FillPattern.HorizontalCylinder,
            fillColor={0,127,0}),
          Text(extent={{-150,-110},{150,-150}}, textColor={0,0,255}, textString="%name")}));
    end Duct;

    model HeatRecoveryUnit
      "Balanced ventilation with a constant-effectiveness heat recovery and constant-pressure fans (trano's system D);
       the reference of its air loop"
      parameter Integer nPorts_a = 1 "Number of extract connections";
      parameter Integer nPorts_b = 1 "Number of supply connections";
      parameter Real m_flow_nominal(unit="kg/s") "Nominal supply flow";
      parameter Real eps = 0.8 "Heat recovery effectiveness";
      parameter Real dpSup_nominal(unit="Pa") = 200 "Pressure rise of the supply fan";
      parameter Real dpRet_nominal(unit="Pa") = 200 "Pressure rise of the return fan";
      parameter Real etaFan = 0.5 "Overall efficiency of the fans";
      parameter Real m_flow_small(unit="kg/s") = 1e-6;
      parameter Real PSup(unit="W") = dpSup_nominal*m_flow_nominal/rhoAir/etaFan "Supply fan power at nominal flow";
      parameter Real PRet(unit="W") = dpRet_nominal*m_flow_nominal/rhoAir/etaFan "Return fan power at nominal flow";
      parameter Real dTFan(unit="K") = (PSup + eps*PRet)/(m_flow_nominal*cpAir)
        "Heating of the supply air by the fans (supply after the exchanger, return before it)";
      AirInlet port_a[nPorts_a] "Extract air"
        annotation (Placement(transformation(extent={{90,-70},{110,-50}})));
      AirOutlet port_b[nPorts_b] "Supply air"
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Interfaces.RealInput y(unit="1") "Relative speed of the fans"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Interfaces.WeatherBusIn weaBus "Weather: the outdoor air"
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Real TExt(unit="K") "Mixed extract air temperature";
      Real TSup(unit="K") "Supply air temperature";
      Real PFan(unit="W") "Electrical power of the fans";
      Real EFan(unit="kWh", start=0, fixed=true) "Electricity meter of the fans";
    equation
      TExt = (sum(port_a[i].m_flow*port_a[i].T for i in 1:nPorts_a) + m_flow_small*sum(port_a[i].T for i in 1:nPorts_a)
        /nPorts_a)/(sum(port_a[i].m_flow for i in 1:nPorts_a) + m_flow_small);
      TSup = weaBus.TDryBul + eps*(TExt - weaBus.TDryBul) + dTFan;
      for i in 1:nPorts_b loop
        port_b[i].T = TSup;
        port_b[i].y = y;
      end for;
      PFan = y*(PSup + PRet);
      der(EFan) = PFan/3.6e6;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,65},{101,55}}, lineColor={0,0,255}, pattern=LinePattern.None,
            fillColor={0,0,255}, fillPattern=FillPattern.Solid),
          Rectangle(extent={{-100,-55},{101,-65}}, lineColor={0,0,255}, pattern=LinePattern.None,
            fillColor={0,127,0}, fillPattern=FillPattern.Solid),
          Rectangle(extent={{-70,78},{70,-80}}, lineColor={0,0,255}, pattern=LinePattern.None,
            fillColor={95,95,95}, fillPattern=FillPattern.Solid),
          Text(extent={{-56,-12},{54,-72}}, textColor={255,255,255}, textString="eps=%eps"),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end HeatRecoveryUnit;
  end Ventilation;

  package Controls
    "Controllers of the closed-loop model: the controls of trano's Buildings models without events (smooth limits
     and set point changes, hysteresis as fast bistable states, anti-windup PI)"
    function smoothMaxN "Smooth maximum of a vector"
      input Real u[:];
      input Real delta = 1e-3;
      output Real y;
    algorithm
      y := u[1];
      for i in 2:size(u, 1) loop
        y := Utilities.smoothMax(y, u[i], delta);
      end for;
    end smoothMaxN;

    model Hysteresis
      "y goes to 1 when u rises above uHigh and back to 0 when u falls below uLow (CDL Hysteresis), as a bistable
       state switching within tau"
      parameter Real uLow "Threshold of the switch to 0";
      parameter Real uHigh "Threshold of the switch to 1";
      parameter Real tau(unit="s") = 10 "Switching time";
      parameter Real y_start = 0 "Initial output";
      Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Real y(start=y_start, fixed=true);
    equation
      tau*der(y) = Utilities.regStep(u - (uHigh + (uLow - uHigh)*y), 1, 0, (uHigh - uLow)/10) - y;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0}, fillColor={210,210,210},
            fillPattern=FillPattern.Solid, borderPattern=BorderPattern.Raised),
          Polygon(points={{-80,90},{-88,68},{-72,68},{-80,90}}, lineColor={192,192,192}, fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Line(points={{-80,68},{-80,-29}}, color={192,192,192}),
          Polygon(points={{92,-29},{70,-21},{70,-37},{92,-29}}, lineColor={192,192,192}, fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Line(points={{-79,-29},{84,-29}}, color={192,192,192}),
          Line(points={{-79,-29},{41,-29}}),
          Line(points={{-15,-21},{1,-29},{-15,-36}}),
          Line(points={{41,51},{41,-29}}),
          Line(points={{33,3},{41,22},{50,3}}),
          Line(points={{-49,51},{81,51}}),
          Line(points={{-4,59},{-19,51},{-4,43}}),
          Line(points={{-59,29},{-49,11},{-39,29}}),
          Line(points={{-49,51},{-49,-29}}),
          Text(extent={{-92,-49},{-9,-92}}, textColor={192,192,192}, textString="%uLow"),
          Text(extent={{2,-49},{91,-92}}, textColor={192,192,192}, textString="%uHigh"),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end Hysteresis;

    model PI
      "PI controller with limits and anti-windup (Modelica LimPID: back-calculation with Ni), its integral relaxed to
       the reset value while disabled"
      parameter Real k "Gain";
      parameter Real Ti(unit="s") "Integral time";
      parameter Real yMin = 0;
      parameter Real yMax = 1;
      parameter Real Ni = 0.9 "Anti-windup gain";
      parameter Real yReset = yMin "Output after a reset";
      parameter Real tauReset(unit="s") = 30 "Time constant of the reset while disabled";
      Interfaces.RealInput e "Control error"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Interfaces.RealInput enable "1: integrate, 0: reset"
        annotation (Placement(transformation(extent={{-20,-20},{20,20}}, rotation=90, origin={0,-120})));
      Real I(start=yReset/k, fixed=true) "Integral of the error";
      Real yUnl "Unlimited output";
      Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));
    equation
      yUnl = k*(e + I);
      y = Utilities.smoothLimit(yUnl, yMin, yMax, 1e-3*(yMax - yMin));
      der(I) = enable*(e/Ti + (y - yUnl)/(k*Ni*Ti)) + (1 - enable)*(yReset/k - e - I)/tauReset;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,-100},{100,100}}, lineColor={0,0,127}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(points={{-80,82},{-88,60},{-72,60},{-80,82}}, lineColor={192,192,192}, fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Line(points={{-80,68},{-80,-100}}, color={192,192,192}),
          Line(points={{-90,-80},{70,-80}}, color={192,192,192}),
          Polygon(points={{74,-80},{52,-72},{52,-88},{74,-80}}, lineColor={192,192,192}, fillColor={192,192,192},
            fillPattern=FillPattern.Solid),
          Line(points={{-80,-80},{-80,-22}}, color={0,0,0}),
          Line(points={{-80,-22},{6,56}}, color={0,0,0}),
          Line(points={{6,56},{68,56}}, color={0,0,0}),
          Text(extent={{-26,-22},{74,-62}}, textColor={0,0,0}, textString="PI"),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end PI;

    partial model PartialEmissionControl
      "Emission control of a zone (trano EmissionControl): set point of its day schedule, output y to the valve or
       the ideal emitter"
      parameter Real schedule[:](each unit="s") = 3600*{7, 19} "Day schedule: occupied between pairs of instants";
      parameter Real THeaSet(unit="K") = 294.15 "Set point when occupied";
      parameter Real THeaSetBack(unit="K") = 289.15 "Set point otherwise";
      parameter Real k = 1 "Gain";
      Interfaces.RealInput TZon(unit="K") "Zone air temperature"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Interfaces.RealOutput y(unit="1") "Valve opening, or share of the power of an ideal emitter"
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));
      Real TSet(unit="K") "Set point";
    equation
      TSet = THeaSetBack + (THeaSet - THeaSetBack)*Utilities.occupancy(Utilities.timeOfDay(time), schedule);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,-100},{100,100}}, lineColor={0,0,127}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end PartialEmissionControl;

    model EmissionP "Variant PController: P controller (IDEAS LimPID) between 0 and 1"
      extends PartialEmissionControl(k=5);
    equation
      y = Utilities.smoothLimit(k*(TSet - TZon), 0, 1, 1e-3);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Text(extent={{-90,40},{90,-40}}, textColor={0,0,0}, textString="P")}));
    end EmissionP;

    model EmissionPI "Variant default: PI heating loop of Buildings G36 ControlLoops (k = 0.1, Ti = 900 s)"
      extends PartialEmissionControl;
      parameter Real kHea = 0.1 "Gain of the heating loop";
      parameter Real TiHea(unit="s") = 900 "Integral time of the heating loop";
      PI conHea(k=kHea, Ti=TiHea, e=TSet - TZon, enable=1);
    equation
      y = conHea.y;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Text(extent={{-90,40},{90,-40}}, textColor={0,0,0}, textString="PI")}));
    end EmissionPI;

    model EmissionOnOff "Variant onoff: on below the set point minus half the band, off above it plus half the band"
      extends PartialEmissionControl;
      parameter Real bandwidth(unit="K") = 0.1 "Band of the on/off controller (Modelica OnOffController)";
      Hysteresis onOff(uLow=-bandwidth/2, uHigh=bandwidth/2, u=TSet - TZon);
    equation
      y = onOff.y;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Text(extent={{-90,40},{90,-40}}, textColor={0,0,0}, textString="on/off")}));
    end EmissionOnOff;

    model EmissionMonitored
      "Variant MonitoredSetpoint: P controller (IDEAS LimPID) between 0 and 1 towards a set point read from external
       data"
      parameter Real k = 1 "Gain";
      Interfaces.RealInput TSet(unit="K") "Set point"
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Interfaces.RealInput TZon(unit="K") "Zone air temperature"
        annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
      Interfaces.RealOutput y(unit="1") "Valve opening, or share of the power of an ideal emitter"
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));
    equation
      y = Utilities.smoothLimit(k*(TSet - TZon), 0, 1, 1e-3);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,-100},{100,100}}, lineColor={0,0,127}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(extent={{-90,40},{90,-40}}, textColor={0,0,0}, textString="P"),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end EmissionMonitored;

    model CollectorControl
      "Collector control of a pump (trano CollectorControl): on when a valve opens beyond 0.5, off when all are
       below 0.01, the speed from a P controller holding the head at dp_nominal"
      parameter Integer nVal = 1 "Number of valves";
      parameter Real k = 1 "Gain";
      Interfaces.RealInput yVal[nVal] "Openings of the valves of the circuit"
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Interfaces.RealInput dpRel "Head of the pump relative to its nominal head"
        annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
      Real yMax "Largest opening";
      Hysteresis hysPum(uLow=0.01, uHigh=0.5, u=yMax);
      Interfaces.RealOutput y(unit="1") "Speed of the pump"
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));
    equation
      yMax = smoothMaxN(yVal);
      y = Utilities.smoothLimit(k*(hysPum.y - dpRel), 0, 1, 1e-3);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,-100},{100,100}}, lineColor={0,0,127}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(extent={{-90,40},{90,-40}}, textColor={0,0,0}, textString="collector"),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end CollectorControl;

    model ThreeWayValveControl
      "Three-way valve control (trano ThreeWayValveControl): P controller on the supply temperature measured
       downstream (filtered by tauSen)"
      parameter Real TColSet(unit="K") = 363.15 "Supply temperature set point";
      parameter Real k = 1 "Gain";
      parameter Real tauSen(unit="s") = 10 "Time constant of the temperature measurement";
      Interfaces.RealInput u(unit="K") "Measured supply temperature"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Interfaces.RealOutput y(unit="1") "Opening of the supply path"
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));
      Real TMea(unit="K", start=TColSet - 30, fixed=true) "Filtered measurement";
    equation
      tauSen*der(TMea) = u - TMea;
      y = Utilities.smoothLimit(k*(TColSet - TMea), 0, 1, 1e-3);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,-100},{100,100}}, lineColor={0,0,127}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(extent={{-90,40},{90,-40}}, textColor={0,0,0}, textString="3WV"),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end ThreeWayValveControl;

    model BoilerDemand
      "Demand of a production (trano BoilerControl, variant simple): on when a valve of its circuits opens beyond 0.5,
       off when all are below 0.01"
      parameter Integer n = 1 "Number of circuits";
      Interfaces.RealInput yMax[n] "Largest opening of the valves of each circuit"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Hysteresis hys(uLow=0.01, uHigh=0.5, u=smoothMaxN(yMax));
      Interfaces.RealOutput y(unit="1") "Demand"
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));
    equation
      y = hys.y;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,-100},{100,100}}, lineColor={0,0,127}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(extent={{-90,40},{90,-40}}, textColor={0,0,0}, textString="demand"),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end BoilerDemand;

    model BoilerTemperatureControl
      "Firing of a boiler without storage (trano Buildings boiler): P controller (k) on the boiler temperature, while
       there is demand"
      parameter Real TempSet(unit="K") = 353.15 "Boiler temperature set point";
      parameter Real k = 1 "Gain";
      Interfaces.RealInput T(unit="K") "Boiler water temperature"
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Interfaces.RealInput demand "Demand, 0 or 1"
        annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
      Interfaces.RealOutput y(unit="1") "Part load ratio"
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));
    equation
      y = demand*Utilities.smoothLimit(k*(TempSet - T), 0, 1, 1e-3);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,-100},{100,100}}, lineColor={0,0,127}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(extent={{-90,40},{90,-40}}, textColor={0,0,0}, textString="boiler"),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end BoilerTemperatureControl;

    model HeatPumpControl
      "Control of trano's air-to-water heat pump (Buildings model): heating curve, heating season, buffer charged
       between TSupSet - dTHys/2 and TSupSet + dTHys/2, compressor PI on the condenser outlet towards TSupSet + dTHys
       (minimum speed yComMin); the minimum on and off times of the safety control are not modelled"
      parameter Real TSet(unit="K") = 323.15 "Supply temperature at the design outdoor temperature";
      parameter Real TSupMin(unit="K") = 303.15 "Supply temperature at the heating limit";
      parameter Real TOutDes(unit="K") = 265.15 "Design outdoor temperature";
      parameter Real TOutHeaLim(unit="K") = 289.15 "Heating limit";
      parameter Real dTHys(unit="K") = 5 "Band of the buffer charging";
      parameter Real kCom = 0.1 "Gain of the compressor controller";
      parameter Real TiCom(unit="s") = 300 "Integral time of the compressor controller";
      parameter Real yComMin = 0.3 "Minimum compressor speed";
      parameter Real tauSen(unit="s") = 30 "Time constant of the condenser outlet temperature";
      Interfaces.RealInput demand "Demand of the circuits, 0 or 1"
        annotation (Placement(transformation(extent={{-140,50},{-100,90}})));
      Interfaces.RealInput TOut(unit="K") "Outdoor air temperature"
        annotation (Placement(transformation(extent={{-140,5},{-100,45}})));
      Interfaces.RealInput TBuf(unit="K") "Buffer temperature"
        annotation (Placement(transformation(extent={{-140,-45},{-100,-5}})));
      Interfaces.RealInput TConOut(unit="K") "Condenser outlet temperature"
        annotation (Placement(transformation(extent={{-140,-90},{-100,-50}})));
      Interfaces.RealOutput y(unit="1") "Compressor speed"
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));
      Real TSupSet(unit="K") "Supply temperature set point (heating curve)";
      Real TCon(unit="K", start=TSet, fixed=true) "Measured condenser outlet temperature";
      Real heaOn "Heating on";
      Hysteresis warOut(uLow=TOutHeaLim - 1, uHigh=TOutHeaLim + 1, u=TOut) "Outside the heating season";
      Hysteresis bufCha(uLow=-dTHys/2, uHigh=dTHys/2, u=TSupSet - TBuf, y_start=1) "Charging the buffer";
      PI conCom(k=kCom, Ti=TiCom, yMin=yComMin, yMax=1, yReset=yComMin, e=TSupSet + dTHys - TCon, enable=heaOn);
    equation
      TSupSet = Utilities.smoothMin(TSet, Utilities.smoothMax(TSupMin, TSet + (TSupMin - TSet)*(TOut - TOutDes)
        /(TOutHeaLim - TOutDes), 0.1), 0.1);
      tauSen*der(TCon) = TConOut - TCon;
      heaOn = demand*(1 - warOut.y)*bufCha.y;
      y = heaOn*conCom.y;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-100,-100},{100,100}}, lineColor={0,0,127}, fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(extent={{-90,40},{90,-40}}, textColor={0,0,0}, textString="heat pump"),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end HeatPumpControl;
  end Controls;

  package Electrical "Electricity production"
    model Photovoltaic "PV array (Buildings PVSimpleOriented): P = A*fAct*eta*eta_DCAC*(HDir + HDif)"
      parameter Real A(unit="m2") "Area of the modules";
      parameter Real eta = 0.12 "Module conversion efficiency";
      parameter Real fAct = 0.9 "Fraction of the area with active cells";
      parameter Real eta_DCAC = 0.9 "Efficiency of the conversion from DC to AC";
      parameter Real til(unit="rad") "Tilt";
      parameter Real azi(unit="rad") "Azimuth: 0 south";
      Interfaces.WeatherBusIn weaBus "Weather"
        annotation (Placement(transformation(extent={{-10,90},{10,110}})));
      BoundaryConditions.TiltedSurface sol(final azi=azi, final til=til) "Irradiance on the modules";
      Real P(unit="W") "Electrical power produced";
      Real EPro(unit="kWh", start=0, fixed=true) "Production meter";
    equation
      connect(weaBus, sol.weaBus);
      P = A*fAct*eta*eta_DCAC*(sol.HDir + sol.HDif);
      der(EPro) = P/3.6e6;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Polygon(points={{-80,-52},{-32,63},{78,63},{29,-52},{-80,-52}}, fillColor={205,203,203},
            fillPattern=FillPattern.Solid, pattern=LinePattern.None, lineColor={0,0,0}),
          Polygon(points={{-69,-45},{-57,-19},{-34,-19},{-45,-45},{-69,-45}}, fillColor={6,13,150},
            fillPattern=FillPattern.Solid, pattern=LinePattern.None),
          Polygon(points={{-53,-9},{-41,17},{-18,17},{-29,-9},{-53,-9}}, fillColor={6,13,150},
            fillPattern=FillPattern.Solid, pattern=LinePattern.None),
          Polygon(points={{-38,27},{-26,53},{-3,53},{-14,27},{-38,27}}, fillColor={6,13,150},
            fillPattern=FillPattern.Solid, pattern=LinePattern.None),
          Polygon(points={{-36,-45},{-24,-19},{-1,-19},{-12,-45},{-36,-45}}, fillColor={6,13,150},
            fillPattern=FillPattern.Solid, pattern=LinePattern.None),
          Polygon(points={{-20,-9},{-8,17},{15,17},{4,-9},{-20,-9}}, fillColor={6,13,150},
            fillPattern=FillPattern.Solid, pattern=LinePattern.None),
          Polygon(points={{-5,27},{7,53},{30,53},{19,27},{-5,27}}, fillColor={6,13,150},
            fillPattern=FillPattern.Solid, pattern=LinePattern.None),
          Polygon(points={{-3,-45},{9,-19},{32,-19},{21,-45},{-3,-45}}, fillColor={6,13,150},
            fillPattern=FillPattern.Solid, pattern=LinePattern.None),
          Polygon(points={{13,-9},{25,17},{48,17},{37,-9},{13,-9}}, fillColor={6,13,150},
            fillPattern=FillPattern.Solid, pattern=LinePattern.None),
          Polygon(points={{28,27},{40,53},{63,53},{52,27},{28,27}}, fillColor={6,13,150},
            fillPattern=FillPattern.Solid, pattern=LinePattern.None),
          Text(extent={{-150,-104},{150,-64}}, textColor={0,0,0}, textString="%name")}));
    end Photovoltaic;
  end Electrical;

  package HVAC "Ideal heating and cooling"
    model IdealHeatingCooling "Heat flow rate Q_flow (an input), split between the room air and its radiant star"
      parameter Real fraRad = 0 "Radiant share";
      Interfaces.RealInput Q_flow(unit="W") "Heat flow rate, positive for heating"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Interfaces.HeatPort heatPortCon "Convective heat to the room air"
        annotation (Placement(transformation(extent={{90,30},{110,50}})));
      Interfaces.HeatPort heatPortRad "Radiative heat to the room surfaces"
        annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
    equation
      heatPortCon.Q_flow = -(1 - fraRad)*Q_flow;
      heatPortRad.Q_flow = -fraRad*Q_flow;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Line(points={{-60,-20},{40,-20}}, color={191,0,0}, thickness=0.5),
          Line(points={{-60,20},{40,20}}, color={191,0,0}, thickness=0.5),
          Line(points={{-80,0},{-60,-20}}, color={191,0,0}, thickness=0.5),
          Line(points={{-80,0},{-60,20}}, color={191,0,0}, thickness=0.5),
          Polygon(points={{40,0},{40,40},{70,20},{40,0}}, lineColor={191,0,0}, fillColor={191,0,0},
            fillPattern=FillPattern.Solid),
          Polygon(points={{40,-40},{40,0},{70,-20},{40,-40}}, lineColor={191,0,0}, fillColor={191,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{70,40},{90,-40}}, lineColor={191,0,0}, fillColor={191,0,0},
            fillPattern=FillPattern.Solid),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end IdealHeatingCooling;

    model IdealHeater
      "Ideal emitter (trano radiator variants ideal and idealbus): Q_flow_nominal times the control y, split between
       the room air and its radiant star"
      parameter Real Q_flow_nominal(unit="W") "Nominal power";
      parameter Real fraRad = 0 "Radiant share";
      Interfaces.RealInput y(unit="1") "Share of the nominal power"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Interfaces.HeatPort heatPortCon "Convective heat to the room air"
        annotation (Placement(transformation(extent={{90,30},{110,50}})));
      Interfaces.HeatPort heatPortRad "Radiative heat to the room surfaces"
        annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
      Real Q_flow(unit="W") "Heat flow rate";
    equation
      Q_flow = Q_flow_nominal*Utilities.smoothLimit(y, 0, 1, 1e-3);
      heatPortCon.Q_flow = -(1 - fraRad)*Q_flow;
      heatPortRad.Q_flow = -fraRad*Q_flow;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Line(points={{-60,-20},{40,-20}}, color={191,0,0}, thickness=0.5),
          Line(points={{-60,20},{40,20}}, color={191,0,0}, thickness=0.5),
          Line(points={{-80,0},{-60,-20}}, color={191,0,0}, thickness=0.5),
          Line(points={{-80,0},{-60,20}}, color={191,0,0}, thickness=0.5),
          Polygon(points={{40,0},{40,40},{70,20},{40,0}}, lineColor={191,0,0}, fillColor={191,0,0},
            fillPattern=FillPattern.Solid),
          Polygon(points={{40,-40},{40,0},{70,-20},{40,-40}}, lineColor={191,0,0}, fillColor={191,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{70,40},{90,-40}}, lineColor={191,0,0}, fillColor={191,0,0},
            fillPattern=FillPattern.Solid),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name")}));
    end IdealHeater;

    model IdealHeatingCoolingPI
      "Ideal heating and cooling of a zone towards day schedules of set points (trano IdealHeatingCooling): a PI
       controller per mode, limited by the capacities"
      parameter Real TSetHea[:, 2] = [0, 293.15] "Heating set point: time of day [s] and set point [K]";
      parameter Real TSetCoo[:, 2] = [0, 300.15] "Cooling set point: time of day [s] and set point [K]";
      parameter Real QHea_flow_max(unit="W") = 1e6 "Heating capacity";
      parameter Real QCoo_flow_max(unit="W") = 1e6 "Cooling capacity";
      parameter Real k = 0.1 "Gain [1/K]";
      parameter Real Ti(unit="s") = 300 "Integral time";
      parameter Real frad = 0 "Radiant share";
      Interfaces.HeatPort heatPortCon "Convective heat to the room air"
        annotation (Placement(transformation(extent={{90,30},{110,50}})));
      Interfaces.HeatPort heatPortRad "Radiative heat to the room surfaces"
        annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
      Real QHea_flow(unit="W") "Heating";
      Real QCoo_flow(unit="W") "Cooling, positive when cooling";
      Real EHea(unit="kWh", start=0, fixed=true) "Heating meter";
      Real ECoo(unit="kWh", start=0, fixed=true) "Cooling meter";
      Controls.PI conHea(k=k, Ti=Ti, e=Utilities.dayTable(Utilities.timeOfDay(time), TSetHea) - heatPortCon.T, enable=1);
      Controls.PI conCoo(k=k, Ti=Ti, e=heatPortCon.T - Utilities.dayTable(Utilities.timeOfDay(time), TSetCoo), enable=1);
    equation
      QHea_flow = conHea.y*QHea_flow_max;
      QCoo_flow = conCoo.y*QCoo_flow_max;
      der(EHea) = QHea_flow/3.6e6;
      der(ECoo) = QCoo_flow/3.6e6;
      heatPortCon.Q_flow = -(1 - frad)*(QHea_flow - QCoo_flow);
      heatPortRad.Q_flow = -frad*(QHea_flow - QCoo_flow);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Line(points={{-60,-20},{40,-20}}, color={191,0,0}, thickness=0.5),
          Line(points={{-60,20},{40,20}}, color={191,0,0}, thickness=0.5),
          Line(points={{-80,0},{-60,-20}}, color={191,0,0}, thickness=0.5),
          Line(points={{-80,0},{-60,20}}, color={191,0,0}, thickness=0.5),
          Polygon(points={{40,0},{40,40},{70,20},{40,0}}, lineColor={191,0,0}, fillColor={191,0,0},
            fillPattern=FillPattern.Solid),
          Polygon(points={{40,-40},{40,0},{70,-20},{40,-40}}, lineColor={191,0,0}, fillColor={191,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{70,40},{90,-40}}, lineColor={191,0,0}, fillColor={191,0,0},
            fillPattern=FillPattern.Solid),
          Text(extent={{-150,150},{150,110}}, textColor={0,0,255}, textString="%name"),
          Text(extent={{-80,80},{20,40}}, textColor={0,0,0}, textString="PI")}));
    end IdealHeatingCoolingPI;
  end HVAC;
end FastModel;
