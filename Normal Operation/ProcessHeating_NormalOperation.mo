model ProcessHeating_NormalOperation
  parameter Modelica.SIunits.PressureDifference my_dp_nominal = 100 "Nominal pressure difference";
  parameter Modelica.SIunits.Pressure p_amb = 10e5 "Ambient pressure";
  IBPSA.Fluid.HeatExchangers.HeaterCooler_u Heater(redeclare package Medium = IBPSA.Media.Air, Q_flow_nominal = 15000, dp_nominal = 0, m_flow_nominal = 1) annotation(
    Placement(visible = true, transformation(extent = {{50, -36}, {70, -16}}, rotation = 0)));
  IBPSA.Fluid.Sensors.MassFlowRate m_in(redeclare package Medium = IBPSA.Media.Air, m_flow_nominal = 0.4) annotation(
    Placement(visible = true, transformation(extent = {{18, -36}, {38, -16}}, rotation = 0)));
  IBPSA.Fluid.Sensors.TemperatureTwoPort T_HX(redeclare package Medium = IBPSA.Media.Air, m_flow_nominal = 0.04) annotation(
    Placement(visible = true, transformation(extent = {{-42, -36}, {-22, -16}}, rotation = 0)));
  Modelica.Blocks.Continuous.LimPID PID(Td = 10, Ti(displayUnit = "s") = 10, k = 50, limitsAtInit = true, yMax = 1, yMin = 0) annotation(
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
  IBPSA.Fluid.MixingVolumes.MixingVolume Chamber(redeclare package Medium = IBPSA.Media.Air, V = 50, m_flow_nominal = 1, nPorts = 4)  annotation(
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
  IBPSA.Fluid.Sources.Boundary_pT bou(redeclare package Medium = IBPSA.Media.Air, nPorts = 1, use_T_in = true) annotation(
    Placement(visible = true, transformation(origin = {174, -78}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  IBPSA.Fluid.Movers.FlowControlled_dp Fan2(redeclare package Medium = IBPSA.Media.Air, addPowerToMedium = false, m_flow_nominal = 1) annotation(
    Placement(visible = true, transformation(origin = {38, -72}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  Modelica.Blocks.Sources.CombiTimeTable T_in_table(extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic, fileName = "C:/Users/guser/TUCloud/MyTUSpace/SIC!/07_Sensor Data Evaluation/Modelica Simulation/HeatingProcessSimulation/Normal Operation/inputSurroundingTemperatures.txt", smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative, tableName = "T_in", tableOnFile = true, timeScale = 60)  annotation(
    Placement(visible = true, transformation(origin = {-148, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.CombiTimeTable T_surr_table(extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic, fileName = "C:/Users/guser/TUCloud/MyTUSpace/SIC!/07_Sensor Data Evaluation/Modelica Simulation/HeatingProcessSimulation/Normal Operation/inputSurroundingTemperatures.txt", smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative,tableName = "T_surr", tableOnFile = true, timeScale = 60)  annotation(
    Placement(visible = true, transformation(origin = {140, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow VariableLoad annotation(
    Placement(visible = true, transformation(origin = {130, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant dist(k = 1)  annotation(
    Placement(visible = true, transformation(origin = {118, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.CombiTimeTable Setpoint_Fan1(extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic, fileName = "C:/Users/guser/TUCloud/MyTUSpace/SIC!/07_Sensor Data Evaluation/Modelica Simulation/HeatingProcessSimulation/Normal Operation/inputSetpoints.txt", tableName = "Setpoint_Fan_in", tableOnFile = true, timeScale = 60)  annotation(
    Placement(visible = true, transformation(origin = {-24, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.CombiTimeTable Setpoint_T(extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic, fileName = "C:/Users/guser/TUCloud/MyTUSpace/SIC!/07_Sensor Data Evaluation/Modelica Simulation/HeatingProcessSimulation/Normal Operation/inputSetpoints.txt", tableName = "Setpoint_Tp", tableOnFile = true, timeScale = 60)  annotation(
    Placement(visible = true, transformation(origin = {98, 64}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.CombiTimeTable Setpoint_Fan2(extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic, fileName = "C:/Users/guser/TUCloud/MyTUSpace/SIC!/07_Sensor Data Evaluation/Modelica Simulation/HeatingProcessSimulation/Normal Operation/inputSetpoints.txt", tableName = "Setpoint_Fan_out", tableOnFile = true, timeScale = 60)  annotation(
    Placement(visible = true, transformation(origin = {-6, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.CombiTimeTable Load(extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic, fileName = "C:/Users/guser/TUCloud/MyTUSpace/SIC!/07_Sensor Data Evaluation/Modelica Simulation/HeatingProcessSimulation/Normal Operation/inputSetpoints.txt", tableName = "Load", tableOnFile = true, timeScale = 60)  annotation(
    Placement(visible = true, transformation(origin = {86, -88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
  connect(bou.ports[1], Chamber.ports[3]) annotation(
    Line(points = {{164, -78}, {164, -48}}, color = {0, 127, 255}));
  connect(Fan2.port_b, T_ext.port_b) annotation(
    Line(points = {{28, -72}, {-28, -72}, {-28, -70}}, color = {0, 127, 255}));
  connect(Fan2.port_a, Chamber.ports[4]) annotation(
    Line(points = {{48, -72}, {122, -72}, {122, -50}, {164, -50}, {164, -48}}, color = {0, 127, 255}));
  connect(T_in_table.y[1], AmbiantAirSource.T_in) annotation(
    Line(points = {{-136, -22}, {-126, -22}, {-126, -22}, {-124, -22}}, color = {0, 0, 127}));
  connect(T_surr_table.y[1], T_surr.T) annotation(
    Line(points = {{152, 68}, {176, 68}, {176, 40}, {176, 40}}, color = {0, 0, 127}));
  connect(bou.T_in, T_surr_table.y[1]) annotation(
    Line(points = {{186, -74}, {196, -74}, {196, 68}, {152, 68}, {152, 68}}, color = {0, 0, 127}));
  connect(VariableLoad.port, Chamber.heatPort) annotation(
    Line(points = {{140, -84}, {156, -84}, {156, -38}, {174, -38}, {174, -38}}, color = {191, 0, 0}));
  connect(dist.y, valveLinear.opening) annotation(
    Line(points = {{129, 2}, {116, 2}, {116, -18}}, color = {0, 0, 127}));
  connect(Setpoint_Fan1.y[1], Fan_in.dp_in) annotation(
    Line(points = {{-12, 44}, {-2, 44}, {-2, -14}, {-2, -14}}, color = {0, 0, 127}));
  connect(Setpoint_T.y[1], PID.u_s) annotation(
    Line(points = {{86, 64}, {78, 64}, {78, 64}, {76, 64}}, color = {0, 0, 127}));
  connect(Setpoint_Fan2.y[1], Fan2.dp_in) annotation(
    Line(points = {{6, -90}, {38, -90}, {38, -84}, {38, -84}}, color = {0, 0, 127}));
  connect(Load.y[1], VariableLoad.Q_flow) annotation(
    Line(points = {{98, -88}, {106, -88}, {106, -84}, {120, -84}, {120, -84}}, color = {0, 0, 127}));
  annotation(
    uses(IBPSA(version = "3.0.0"), Modelica(version = "3.2.3")),
    Diagram(coordinateSystem(extent = {{-160, -100}, {200, 100}})),
    Icon(coordinateSystem(extent = {{-160, -100}, {200, 100}})),
    experiment(StartTime = 0, StopTime = 86400, Tolerance = 1e-06, Interval = 60));
end ProcessHeating_NormalOperation;
