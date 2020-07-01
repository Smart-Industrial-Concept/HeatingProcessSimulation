model ProcessHeating_v1_2b
  parameter Modelica.SIunits.PressureDifference my_dp_nominal = 100 "Nominal pressure difference";
  parameter Modelica.SIunits.Pressure p_amb = 10e5 "Ambient pressure";
  IBPSA.Fluid.HeatExchangers.HeaterCooler_u Heater(redeclare package Medium = IBPSA.Media.Air, Q_flow_nominal = 20000, dp_nominal = 0, m_flow_nominal = 1) annotation(
    Placement(visible = true, transformation(extent = {{50, -36}, {70, -16}}, rotation = 0)));
  IBPSA.Fluid.Sensors.MassFlowRate m_in(redeclare package Medium = IBPSA.Media.Air, m_flow_nominal = 0.4) annotation(
    Placement(visible = true, transformation(extent = {{18, -36}, {38, -16}}, rotation = 0)));
  IBPSA.Fluid.Sensors.TemperatureTwoPort T_HX(redeclare package Medium = IBPSA.Media.Air, m_flow_nominal = 0.04) annotation(
    Placement(visible = true, transformation(extent = {{-42, -36}, {-22, -16}}, rotation = 0)));
  Modelica.Blocks.Continuous.LimPID PID(Td = 0.1, Ti(displayUnit = "s") = 10, k = 50, limitsAtInit = true, yMax = 1, yMin = 0) annotation(
    Placement(visible = true, transformation(extent = {{74, 54}, {54, 74}}, rotation = 0)));
  IBPSA.Fluid.Sensors.Temperature T_control(redeclare package Medium = IBPSA.Media.Air) annotation(
    Placement(visible = true, transformation(extent = {{100, -12}, {80, 8}}, rotation = 0)));
  IBPSA.Fluid.HeatExchangers.ConstantEffectiveness HeatExchanger(redeclare package Medium1 = IBPSA.Media.Air, redeclare package Medium2 = IBPSA.Media.Air, allowFlowReversal1 = true, allowFlowReversal2 = false, dp1_nominal = 20, dp2_nominal = 20, m1_flow_nominal = 1, m2_flow_nominal = 1, show_T = true) annotation(
    Placement(visible = true, transformation(origin = {-58, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  IBPSA.Fluid.Movers.FlowControlled_dp Fan_in(redeclare package Medium = IBPSA.Media.Air, addPowerToMedium = false, allowFlowReversal = true, dp_nominal = 100, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, inputType = IBPSA.Fluid.Types.InputType.Continuous, m_flow_nominal = 0.1, nominalValuesDefineDefaultPressureCurve = true) annotation(
    Placement(visible = true, transformation(origin = {-2, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Valves.ValveLinear valveLinear(redeclare package Medium = IBPSA.Media.Air, dp_nominal = 500, m_flow_nominal = 1) annotation(
    Placement(visible = true, transformation(origin = {116, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  IBPSA.Fluid.Sensors.TemperatureTwoPort T_sup(redeclare package Medium = IBPSA.Media.Air, m_flow_nominal = 0.04) annotation(
    Placement(visible = true, transformation(extent = {{138, -36}, {158, -16}}, rotation = 0)));
  IBPSA.Fluid.Sensors.TemperatureTwoPort T_in(redeclare package Medium = IBPSA.Media.Air, m_flow_nominal = 0.04) annotation(
    Placement(visible = true, transformation(extent = {{-92, -36}, {-72, -16}}, rotation = 0)));
  IBPSA.Fluid.Sources.Boundary_pT AmbiantAirSource(redeclare package Medium = IBPSA.Media.Air, nPorts = 1, use_T_in = true) annotation(
    Placement(visible = true, transformation(extent = {{-122, -36}, {-102, -16}}, rotation = 0)));
  IBPSA.Fluid.Sensors.TemperatureTwoPort T_exh(redeclare package Medium = IBPSA.Media.Air, m_flow_nominal = 0.1) annotation(
    Placement(visible = true, transformation(extent = {{-88, -80}, {-68, -60}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine Setpoint_T_in(amplitude = 3, freqHz = 1 / 4000, offset = 10 + 273.15, startTime = 300) annotation(
    Placement(visible = true, transformation(origin = {-134, 6}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.TimeTable disturbance(table = [0, 1; 500,1; 500,0.6;800,0.6;800,1; 1900, 1; 2000, 0.7; 2200, 0.7; 2300, 1;3000,1;5000,0.4; 6000, 1; 6100, 1]) annotation(
    Placement(visible = true, transformation(origin = {-24, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  IBPSA.Fluid.MixingVolumes.MixingVolume Chamber(redeclare package Medium = IBPSA.Media.Air, V = 30, m_flow_nominal = 1, nPorts = 4)  annotation(
    Placement(visible = true, transformation(origin = {174, -48}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor(G = 30)  annotation(
    Placement(visible = true, transformation(origin = {176, -4}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  IBPSA.Fluid.Sources.Boundary_pT AirSink2(nPorts = 1, redeclare package Medium = IBPSA.Media.Air) annotation(
    Placement(visible = true, transformation(origin = {-136, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  IBPSA.Fluid.Sensors.TemperatureTwoPort T_ext(redeclare package Medium = IBPSA.Media.Air, m_flow_nominal = 0.1) annotation(
    Placement(visible = true, transformation(extent = {{-48, -80}, {-28, -60}}, rotation = 0)));
  IBPSA.Fluid.Sensors.MassFlowRate m_out(redeclare package Medium = IBPSA.Media.Air, m_flow_nominal = 0.4) annotation(
    Placement(visible = true, transformation(origin = {-108, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature T_surr annotation(
    Placement(visible = true, transformation(origin = {176, 28}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.Sine Setpoint_T_surr(amplitude = 2, freqHz = 1 / 2000, offset = 12 + 273.15, startTime = 300) annotation(
    Placement(visible = true, transformation(origin = {176, 62}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.TimeTable setpoint_T_control(offset = 273.15, table = [0, 20; 1000, 20; 1000, 23; 2600, 23; 2600, 26; 5000, 26; 5000, 23; 6500, 23; 6500, 20; 8000, 20]) annotation(
    Placement(visible = true, transformation(origin = {96, 64}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.TimeTable Setpoint_dpFan_inlet(offset = 0, table = [0, 500; 1200, 500; 1200, 400; 1500, 400; 1500, 500; 6500, 500; 6500, 300; 7500, 300; 7500, 500; 8000, 500]) annotation(
    Placement(visible = true, transformation(origin = {-2, 8}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  IBPSA.Fluid.Sources.Boundary_pT bou(redeclare package Medium = IBPSA.Media.Air, nPorts = 1, use_T_in = true) annotation(
    Placement(visible = true, transformation(origin = {174, -78}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = 5)  annotation(
    Placement(visible = true, transformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  IBPSA.Fluid.Movers.FlowControlled_dp Fan2(redeclare package Medium = IBPSA.Media.Air, addPowerToMedium = false, m_flow_nominal = 1) annotation(
    Placement(visible = true, transformation(origin = {38, -72}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
equation
  connect(T_control.T, PID.u_m) annotation(
    Line(points = {{83, -2}, {64, -2}, {64, 52}}, color = {0, 0, 127}));
  connect(PID.y, Heater.u) annotation(
    Line(points = {{53, 64}, {48, 64}, {48, -20}}, color = {0, 0, 127}));
  connect(T_in.port_a, AmbiantAirSource.ports[1]) annotation(
    Line(points = {{-92, -26}, {-102, -26}}, color = {0, 127, 255}));
  connect(T_exh.port_b, HeatExchanger.port_b2) annotation(
    Line(points = {{-68, -70}, {-68, -38}}, color = {0, 127, 255}));
  connect(valveLinear.port_b, T_sup.port_a) annotation(
    Line(points = {{126, -26}, {138, -26}}, color = {0, 127, 255}));
  connect(AmbiantAirSource.T_in, Setpoint_T_in.y) annotation(
    Line(points = {{-124, -22}, {-133.5, -22}, {-133.5, -5}, {-134, -5}}, color = {0, 0, 127}));
  connect(Chamber.heatPort, thermalConductor.port_b) annotation(
    Line(points = {{174, -38}, {175, -38}, {175, -14}, {176, -14}}, color = {191, 0, 0}));
  connect(T_sup.port_b, Chamber.ports[1]) annotation(
    Line(points = {{158, -26}, {165, -26}, {165, -48}, {164, -48}}));
  connect(T_ext.port_a, HeatExchanger.port_a2) annotation(
    Line(points = {{-48, -70}, {-48, -38}}, color = {0, 127, 255}));
  connect(T_in.port_b, HeatExchanger.port_a1) annotation(
    Line(points = {{-72, -26}, {-68, -26}}, color = {0, 127, 255}));
  connect(Fan_in.port_b, m_in.port_a) annotation(
    Line(points = {{8, -26}, {18, -26}}, color = {0, 127, 255}));
  connect(m_in.port_b, Heater.port_a) annotation(
    Line(points = {{38, -26}, {50, -26}}, color = {0, 127, 255}));
  connect(Heater.port_b, valveLinear.port_a) annotation(
    Line(points = {{70, -26}, {106, -26}}, color = {0, 127, 255}));
  connect(HeatExchanger.port_b1, T_HX.port_a) annotation(
    Line(points = {{-48, -26}, {-42, -26}}, color = {0, 127, 255}));
  connect(T_HX.port_b, Fan_in.port_a) annotation(
    Line(points = {{-22, -26}, {-12, -26}}, color = {0, 127, 255}));
  connect(m_out.port_b, AirSink2.ports[1]) annotation(
    Line(points = {{-118, -70}, {-126, -70}}, color = {0, 127, 255}));
  connect(T_surr.port, thermalConductor.port_a) annotation(
    Line(points = {{176, 18}, {176, 6}}, color = {191, 0, 0}));
  connect(T_control.port, Chamber.ports[2]) annotation(
    Line(points = {{90, -12}, {90, -48}, {164, -48}}, color = {0, 127, 255}));
  connect(T_exh.port_a, m_out.port_a) annotation(
    Line(points = {{-88, -70}, {-98, -70}}, color = {0, 127, 255}));
  connect(Setpoint_T_surr.y, T_surr.T) annotation(
    Line(points = {{176, 51}, {176, 40}}, color = {0, 0, 127}));
  connect(setpoint_T_control.y, PID.u_s) annotation(
    Line(points = {{85, 64}, {76, 64}}, color = {0, 0, 127}));
  connect(Fan_in.dp_in, Setpoint_dpFan_inlet.y) annotation(
    Line(points = {{-2, -14}, {-2, -3}}, color = {0, 0, 127}));
  connect(bou.T_in, Setpoint_T_surr.y) annotation(
    Line(points = {{186, -74}, {186, -61.5}, {192, -61.5}, {192, -69}, {194, -69}, {194, 50}, {176, 50}, {176, 51}}, color = {0, 0, 127}));
  connect(bou.ports[1], Chamber.ports[3]) annotation(
    Line(points = {{164, -78}, {164, -48}}, color = {0, 127, 255}));
  connect(Fan2.port_b, T_ext.port_b) annotation(
    Line(points = {{28, -72}, {-28, -72}, {-28, -70}}, color = {0, 127, 255}));
  connect(const1.y, Fan2.dp_in) annotation(
    Line(points = {{12, -90}, {38, -90}, {38, -84}, {38, -84}}, color = {0, 0, 127}));
  connect(Fan2.port_a, Chamber.ports[4]) annotation(
    Line(points = {{48, -72}, {122, -72}, {122, -50}, {164, -50}, {164, -48}}, color = {0, 127, 255}));
  connect(disturbance.y, valveLinear.opening) annotation(
    Line(points = {{-12, 84}, {22, 84}, {22, 18}, {116, 18}, {116, -18}, {116, -18}}, color = {0, 0, 127}));
  annotation(
    uses(IBPSA(version = "3.0.0"), Modelica(version = "3.2.3")),
    Diagram(coordinateSystem(extent = {{-160, -100}, {200, 100}})),
    Icon(coordinateSystem(extent = {{-160, -100}, {200, 100}})),
    experiment(StartTime = 0, StopTime = 8000, Tolerance = 1e-06, Interval = 1));
end ProcessHeating_v1_2b;
