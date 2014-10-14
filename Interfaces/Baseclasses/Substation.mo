within DistrictHeating4G.Interfaces.Baseclasses;
partial model Substation "Interface for a local substation"

  IDEAS.Fluid.Interfaces.FlowPort_b flowPort_return_b
    annotation (Placement(transformation(extent={{-110,-30},{-90,-10}})));
  IDEAS.Fluid.Interfaces.FlowPort_a flowPort_supply_a
    annotation (Placement(transformation(extent={{-110,10},{-90,30}})));
  IDEAS.Fluid.Interfaces.FlowPort_b flowPort_b1
    annotation (Placement(transformation(extent={{-30,90},{-10,110}})));
  IDEAS.Fluid.Interfaces.FlowPort_a flowPort_return_a
    annotation (Placement(transformation(extent={{90,-30},{110,-10}})));
  IDEAS.Fluid.Interfaces.FlowPort_b flowPort_supply_b
    annotation (Placement(transformation(extent={{90,10},{110,30}})));
  IDEAS.Fluid.Interfaces.FlowPort_a flowPort_a1
    annotation (Placement(transformation(extent={{10,90},{30,110}})));
  replaceable IDEAS.Fluid.Interfaces.StaticFourPortHeatMassExchanger
    staticFourPortHeatMassExchanger
    annotation (Placement(transformation(extent={{-10,68},{10,88}})));
  IDEAS.Fluid.FixedResistances.SplitterFixedResistanceDpM spl1(
    m_flow_nominal={0.1,-0.1,-0.1},
    dp_nominal={20,20,10},
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater)
    annotation (Placement(transformation(extent={{-10,-10},{-30,-30}})));
  IDEAS.Fluid.FixedResistances.SplitterFixedResistanceDpM spl2(
    m_flow_nominal={0.1,-0.1,-0.1},
    dp_nominal={20,20,10},
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater)
    annotation (Placement(transformation(extent={{30,30},{10,10}})));
  replaceable IDEAS.Fluid.FixedResistances.Pipe_Insulated pipe_Insulated annotation (
      Placement(transformation(
        extent={{-10,-4},{10,4}},
        rotation=180,
        origin={-66,-20})));
  replaceable IDEAS.Fluid.FixedResistances.Pipe_Insulated pipe_Insulated1 annotation (
      Placement(transformation(
        extent={{-10,-4},{10,4}},
        rotation=180,
        origin={-66,20})));
  replaceable IDEAS.Fluid.Movers.Pump pump(useInput=true)
                                           annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-20,42})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-40,-60})));
  Modelica.Blocks.Interfaces.RealInput u "Outside temperature" annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-106})));
equation
  connect(flowPort_b1, staticFourPortHeatMassExchanger.port_a1) annotation (
      Line(
      points={{-20,100},{-20,84},{-10,84}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(staticFourPortHeatMassExchanger.port_b1, flowPort_a1) annotation (
      Line(
      points={{10,84},{20,84},{20,100}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(staticFourPortHeatMassExchanger.port_a2, spl2.port_3) annotation (
      Line(
      points={{10,72},{20,72},{20,30}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl2.port_1, flowPort_supply_b) annotation (Line(
      points={{30,20},{100,20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl1.port_1, flowPort_return_a) annotation (Line(
      points={{-10,-20},{100,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl1.port_2, pipe_Insulated.port_a) annotation (Line(
      points={{-30,-20},{-56,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated.port_b, flowPort_return_b) annotation (Line(
      points={{-76,-20},{-100,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl2.port_2, pipe_Insulated1.port_a) annotation (Line(
      points={{10,20},{-56,20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated1.port_b, flowPort_supply_a) annotation (Line(
      points={{-76,20},{-100,20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(staticFourPortHeatMassExchanger.port_b2, pump.port_a) annotation (
      Line(
      points={{-10,72},{-20,72},{-20,52}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump.port_b, spl1.port_3) annotation (Line(
      points={{-20,32},{-20,-10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated.heatPort, prescribedTemperature.port) annotation (Line(
      points={{-66,-16},{-66,-8},{-40,-8},{-40,-50}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(pipe_Insulated1.heatPort, prescribedTemperature.port) annotation (
      Line(
      points={{-66,24},{-66,32},{-40,32},{-40,-50}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(prescribedTemperature.T, u) annotation (Line(
      points={{-40,-72},{-40,-80},{0,-80},{0,-106}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(graphics={
        Line(
          points={{0,12},{7.3479e-016,-24}},
          color={0,0,255},
          smooth=Smooth.None,
          origin={-82,-20},
          rotation=90),
        Rectangle(
          extent={{-58,60},{62,-60}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-58,72},{62,48}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-58,-48},{62,-72}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-20,62},{-20,98}},
          color={0,0,255},
          smooth=Smooth.None),
        Line(
          points={{20,62},{20,98}},
          color={0,0,255},
          smooth=Smooth.None),
        Line(
          points={{0,12},{7.3479e-016,-24}},
          color={0,0,255},
          smooth=Smooth.None,
          origin={-82,20},
          rotation=90),
        Line(
          points={{0,12},{7.3479e-016,-24}},
          color={0,0,255},
          smooth=Smooth.None,
          origin={74,20},
          rotation=90),
        Line(
          points={{0,12},{7.3479e-016,-24}},
          color={0,0,255},
          smooth=Smooth.None,
          origin={74,-20},
          rotation=90)}));
end Substation;
