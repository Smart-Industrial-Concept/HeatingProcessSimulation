model ProcessHeating_withVaribaleInput
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
  IBPSA.Fluid.Sources.Boundary_pT bou(redeclare package Medium = IBPSA.Media.Air, nPorts = 1, use_T_in = true) annotation(
    Placement(visible = true, transformation(origin = {174, -78}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  IBPSA.Fluid.Movers.FlowControlled_dp Fan2(redeclare package Medium = IBPSA.Media.Air, addPowerToMedium = false, m_flow_nominal = 1) annotation(
    Placement(visible = true, transformation(origin = {38, -72}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  Modelica.Blocks.Sources.CombiTimeTable setpointFan1(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, fileName = "C:/Users/guser/TUCloud/MyTUSpace/SIC!/07_Sensor Data Evaluation/Modelica Simulation/HeatingProcessSimulation/Varibale Operation/input_forParameterIdentification.txt", tableName = "setpointFan1_pressure", tableOnFile = true)  annotation(
    Placement(visible = true, transformation(origin = {-40, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, fileName = "C:/Users/guser/TUCloud/MyTUSpace/SIC!/07_Sensor Data Evaluation/Modelica Simulation/HeatingProcessSimulation/Varibale Operation/input_forParameterIdentification.txt", tableName = "T_inletAir", tableOnFile = true) annotation(
    Placement(visible = true, transformation(origin = {-150, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable1(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, fileName = "C:/Users/guser/TUCloud/MyTUSpace/SIC!/07_Sensor Data Evaluation/Modelica Simulation/HeatingProcessSimulation/Varibale Operation/input_forParameterIdentification.txt", tableName = "setpointControl", tableOnFile = true) annotation(
    Placement(visible = true, transformation(origin = {116, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 1)  annotation(
    Placement(visible = true, transformation(origin = {110, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable2(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, fileName = "C:/Users/guser/TUCloud/MyTUSpace/SIC!/07_Sensor Data Evaluation/Modelica Simulation/HeatingProcessSimulation/Varibale Operation/input_forParameterIdentification.txt", tableName = "T_surr", tableOnFile = true) annotation(
    Placement(visible = true, transformation(origin = {158, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable3(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, fileName = "C:/Users/guser/TUCloud/MyTUSpace/SIC!/07_Sensor Data Evaluation/Modelica Simulation/HeatingProcessSimulation/Varibale Operation/input_forParameterIdentification.txt", tableName = "setpointFan2_pressure", tableOnFile = true) annotation(
    Placement(visible = true, transformation(origin = {-16, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
  connect(setpointFan1.y[1], Fan_in.dp_in) annotation(
    Line(points = {{-28, 30}, {-2, 30}, {-2, -14}, {-2, -14}}, color = {0, 0, 127}));
  connect(combiTimeTable.y[1], AmbiantAirSource.T_in) annotation(
    Line(points = {{-138, -10}, {-134, -10}, {-134, -22}, {-124, -22}, {-124, -22}}, color = {0, 0, 127}));
  connect(PID.u_s, combiTimeTable1.y[1]) annotation(
    Line(points = {{76, 64}, {88, 64}, {88, 66}, {128, 66}, {128, 66}}, color = {0, 0, 127}));
  connect(valveLinear.opening, const.y) annotation(
    Line(points = {{116, -18}, {122, -18}, {122, 18}, {122, 18}}, color = {0, 0, 127}));
  connect(combiTimeTable2.y[1], T_surr.T) annotation(
    Line(points = {{170, 80}, {176, 80}, {176, 40}, {176, 40}}, color = {0, 0, 127}));
  connect(bou.T_in, combiTimeTable2.y[1]) annotation(
    Line(points = {{186, -74}, {192, -74}, {192, 80}, {170, 80}, {170, 80}}, color = {0, 0, 127}));
  connect(combiTimeTable3.y[1], Fan2.dp_in) annotation(
    Line(points = {{-4, -92}, {38, -92}, {38, -84}, {38, -84}}, color = {0, 0, 127}));
  annotation(
    uses(IBPSA(version = "3.0.0"), Modelica(version = "3.2.3")),
    Diagram(coordinateSystem(extent = {{-160, -100}, {200, 100}})),
    Icon(coordinateSystem(extent = {{-160, -100}, {200, 100}})),
    experiment(StartTime = 0, StopTime = 8000, Tolerance = 1e-06, Interval = 1));
end ProcessHeating_withVaribaleInput;
