within DistrictHeating4G.Substations;
model HXWithBypass
  //Extensions
  extends DistrictHeating4G.Interfaces.Baseclasses.Substation(
    flowPort_supply_out(redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater),
    flowPort_return_in(redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater),
    flowPort_b1(redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater),
    flowPort_a1(redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater),
    flowPort_supply_in(redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater),
    flowPort_return_out(redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater));

  //Components
  replaceable IDEAS.Fluid.HeatExchangers.ConstantEffectiveness
    staticFourPortHeatMassExchanger(
    redeclare package Medium1 =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    redeclare package Medium2 =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m1_flow_nominal=0.1,
    m2_flow_nominal=0.1,
    dp1_nominal=2,
    dp2_nominal=2) constrainedby
    IDEAS.Fluid.Interfaces.StaticFourPortHeatMassExchanger(
    redeclare package Medium1 =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    redeclare package Medium2 =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m1_flow_nominal=0.1,
    m2_flow_nominal=0.1,
    dp1_nominal=2,
    dp2_nominal=2) "Building substation"
    annotation (Placement(transformation(extent={{-10,66},{10,86}})));
  IDEAS.Fluid.FixedResistances.SplitterFixedResistanceDpM spl1(
    m_flow_nominal={0.1,-0.1,-0.1},
    dp_nominal={20,20,10},
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater) "Splitter for bypass"
    annotation (Placement(transformation(extent={{-16,-10},{-36,-30}})));
  IDEAS.Fluid.FixedResistances.SplitterFixedResistanceDpM spl2(
    m_flow_nominal={0.1,-0.1,-0.1},
    dp_nominal={20,20,10},
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater) "Splitter for bypass"
    annotation (Placement(transformation(extent={{80,30},{60,10}})));
  replaceable IDEAS.Fluid.FixedResistances.Pipe_Insulated pipe_Insulated(
    redeclare package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
    m_flow_nominal=0.2,
    m=5,
    UA=10) "Insulated pipe with heat exchange to the outside"
    annotation (
      Placement(transformation(
        extent={{-10,-4},{10,4}},
        rotation=180,
        origin={-66,-20})));

  replaceable IDEAS.Fluid.FixedResistances.Pipe_Insulated pipe_Insulated1(
    redeclare package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
    m_flow_nominal=0.2,
    UA=10,
    m=5) "Insulated pipe with heat exchange to the outside"
    annotation (
      Placement(transformation(
        extent={{-10,-4},{10,4}},
        rotation=180,
        origin={-66,20})));

  replaceable IDEAS.Fluid.Movers.Pump pump(useInput=true,
    redeclare package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
    m_flow_nominal=0.1) "Pump of the heat exchanger"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={70,44})));

  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature "Ambient Temperature"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-66,-54})));

  Modelica.Fluid.Sensors.TemperatureTwoPort temperature(redeclare package
      Medium = Modelica.Media.Water.ConstantPropertyLiquidWater)
    "Sensor of the return temperature" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-26,44})));
  IDEAS.Controls.Continuous.LimPID conPID(controllerType=Modelica.Blocks.Types.SimpleController.PI)
    annotation (Placement(transformation(extent={{30,34},{50,54}})));
  Modelica.Blocks.Sources.Constant const(k=273.15 + 50)
    "Temperature below wich the pump should be on"
    annotation (Placement(transformation(extent={{-6,34},{14,54}})));
equation
  //Connections
  connect(flowPort_b1, staticFourPortHeatMassExchanger.port_a1) annotation (
      Line(
      points={{-20,100},{-20,82},{-10,82}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(staticFourPortHeatMassExchanger.port_b1, flowPort_a1) annotation (
      Line(
      points={{10,82},{20,82},{20,100}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl1.port_2, pipe_Insulated.port_a) annotation (Line(
      points={{-36,-20},{-56,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl2.port_2, pipe_Insulated1.port_a) annotation (Line(
      points={{60,20},{-56,20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated.heatPort, prescribedTemperature.port) annotation (Line(
      points={{-66,-16},{-66,-8},{-44,-8},{-44,-30},{-66,-30},{-66,-44}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(pipe_Insulated1.heatPort, prescribedTemperature.port) annotation (
      Line(
      points={{-66,24},{-66,40},{-44,40},{-44,-30},{-66,-30},{-66,-44}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(prescribedTemperature.T, u) annotation (Line(
      points={{-66,-66},{-66,-80},{0,-80},{0,-106}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(flowPort_supply_in, flowPort_supply_in) annotation (Line(
      points={{100,20},{100,20}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(spl2.port_1, flowPort_supply_in) annotation (Line(
      points={{80,20},{100,20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated1.port_b, flowPort_supply_out) annotation (Line(
      points={{-76,20},{-100,20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl1.port_1, flowPort_return_out) annotation (Line(
      points={{-16,-20},{100,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated.port_b, flowPort_return_in) annotation (Line(
      points={{-76,-20},{-100,-20}},
      color={0,127,255},
      smooth=Smooth.None));

  connect(staticFourPortHeatMassExchanger.port_a2, pump.port_b) annotation (
      Line(
      points={{10,70},{70,70},{70,54}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl2.port_3, pump.port_a) annotation (Line(
      points={{70,30},{70,34}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(staticFourPortHeatMassExchanger.port_b2, temperature.port_a)
    annotation (Line(
      points={{-10,70},{-26,70},{-26,54}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(temperature.T, conPID.u_m) annotation (Line(
      points={{-15,44},{-10,44},{-10,24},{40,24},{40,32}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(conPID.y, pump.m_flowSet) annotation (Line(
      points={{51,44},{59.6,44}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(conPID.u_s, const.y) annotation (Line(
      points={{28,44},{15,44}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(temperature.port_b, spl1.port_3) annotation (Line(
      points={{-26,34},{-26,-10}},
      color={0,127,255},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics));
end HXWithBypass;
