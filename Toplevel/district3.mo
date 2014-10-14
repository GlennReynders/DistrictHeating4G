within DistrictHeating.Toplevel;
model district3

  inner IDEAS.SimInfoManager sim(occBeh=false, DHW=false)
    annotation (Placement(transformation(extent={{-98,70},{-78,90}})));
  inner Modelica.Fluid.System system
    annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
  DistrictHeating.Interfaces.Building building(
    redeclare IDEAS.HeatingSystems.Examples.DummyBuilding building(nZones=1,
        nEmb=0),
    redeclare IDEAS.VentilationSystems.None ventilationSystem,
    redeclare IDEAS.Occupants.Standards.ISO13790 occupant,
    DH=true,
    redeclare HeatingSystem.Radiator                        heatingSystem(
        DH=true,
      port_a(redeclare package Medium =
            Modelica.Media.Water.ConstantPropertyLiquidWater),
      port_b(redeclare package Medium =
            Modelica.Media.Water.ConstantPropertyLiquidWater)))
    annotation (Placement(transformation(extent={{-74,36},{-54,56}})));

  DistrictHeating.Interfaces.Building building1(
    redeclare IDEAS.HeatingSystems.Examples.DummyBuilding building(nZones=1,
        nEmb=0),
    redeclare IDEAS.VentilationSystems.None ventilationSystem,
    redeclare IDEAS.Occupants.Standards.ISO13790 occupant,
    DH=true,
    redeclare HeatingSystem.Radiator                        heatingSystem(
        DH=true,
      port_a(redeclare package Medium =
            Modelica.Media.Water.ConstantPropertyLiquidWater),
      port_b(redeclare package Medium =
            Modelica.Media.Water.ConstantPropertyLiquidWater)))
    annotation (Placement(transformation(extent={{4,32},{24,52}})));
  IDEAS.Fluid.Production.Boiler boiler(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    QNom=10000,
    mWater=50,
    m_flow_nominal=0.1) annotation (Placement(transformation(
        extent={{-10,-11},{10,11}},
        rotation=-90,
        origin={54,33})));
  IDEAS.Fluid.FixedResistances.Pipe_Insulated pipe_Insulated(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m=1,
    UA=10,
    m_flow_nominal=0.2,
    dp_nominal=30) annotation (Placement(transformation(
        extent={{-13,-5},{13,5}},
        rotation=-90,
        origin={69,-1})));
  IDEAS.Fluid.FixedResistances.Pipe_Insulated pipe_Insulated1(
    m=1,
    UA=10,
    m_flow_nominal=0.1,
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater)
    annotation (Placement(transformation(extent={{4,-50},{24,-42}})));
  IDEAS.Fluid.Movers.Pump pump(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, m_flow_nominal=
        0.2)
    annotation (Placement(transformation(extent={{46,-56},{24,-34}})));
  IDEAS.Fluid.FixedResistances.Pipe_Insulated pipe_Insulated2(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m=1,
    UA=10,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{24,8},{44,16}})));
  IDEAS.Fluid.HeatExchangers.ConstantEffectiveness hex(
    redeclare package Medium1 =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    redeclare package Medium2 =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m1_flow_nominal=0.1,
    m2_flow_nominal=0.1,
    dp1_nominal=30,
    dp2_nominal=30)
    annotation (Placement(transformation(extent={{-14,-22},{6,-2}})));
  IDEAS.Fluid.HeatExchangers.ConstantEffectiveness hex1(
    redeclare package Medium1 =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m1_flow_nominal=0.1,
    m2_flow_nominal=0.1,
    dp1_nominal=20,
    dp2_nominal=20,
    redeclare package Medium2 =
        Modelica.Media.Water.ConstantPropertyLiquidWater)
    annotation (Placement(transformation(extent={{-70,-22},{-50,-2}})));
  IDEAS.Fluid.FixedResistances.SplitterFixedResistanceDpM spl(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m_flow_nominal={0.2,-0.1,-0.1},
    dp_nominal={20,20,10})
    annotation (Placement(transformation(extent={{4,-36},{-16,-56}})));
  IDEAS.Fluid.FixedResistances.SplitterFixedResistanceDpM spl1(
    m_flow_nominal={0.1,-0.1,-0.1},
    dp_nominal={20,20,10},
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater)
    annotation (Placement(transformation(extent={{-52,-36},{-72,-56}})));
  IDEAS.Fluid.FixedResistances.Pipe_Insulated pipe_Insulated3(
    m=1,
    UA=10,
    m_flow_nominal=0.1,
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater)
    annotation (Placement(transformation(extent={{-44,-48},{-24,-40}})));
  IDEAS.Fluid.FixedResistances.Pipe_Insulated pipe_Insulated4(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    dp_nominal=10,
    m=1,
    UA=10,
    m_flow_nominal=0.1) annotation (Placement(transformation(
        extent={{-10,-4},{10,4}},
        rotation=90,
        origin={-90,-14})));
  IDEAS.Fluid.FixedResistances.SplitterFixedResistanceDpM spl2(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m_flow_nominal={0.1,0.1,-0.2},
    dp_nominal={20,20,10})
    annotation (Placement(transformation(extent={{-54,0},{-34,20}})));
  IDEAS.Fluid.FixedResistances.Pipe_Insulated pipe_Insulated5(
    m=1,
    UA=10,
    m_flow_nominal=0.1,
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater)
    annotation (Placement(transformation(extent={{-28,6},{-8,14}})));
  IDEAS.Fluid.FixedResistances.SplitterFixedResistanceDpM spl3(
    m_flow_nominal={0.1,0.1,-0.2},
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    dp_nominal={20,20,10})
    annotation (Placement(transformation(extent={{-8,0},{12,20}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=350)
    annotation (Placement(transformation(extent={{48,48},{68,68}})));
equation
  connect(boiler.port_b, pipe_Insulated.port_a) annotation (Line(
      points={{58,22},{70,22},{70,12},{69,12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated.port_b, pump.port_a) annotation (Line(
      points={{69,-14},{69,-12},{46,-12},{46,-45}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump.port_b, pipe_Insulated1.port_b) annotation (Line(
      points={{24,-45},{24,-46}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(boiler.port_a, pipe_Insulated2.port_b) annotation (Line(
      points={{50,22},{42,22},{42,12},{44,12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building1.port_a, hex.port_a1) annotation (Line(
      points={{12.4,32},{-10,32},{-10,-6},{-14,-6}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building1.port_b, hex.port_b1) annotation (Line(
      points={{15.3,32.1},{15.3,5.05},{6,5.05},{6,-6}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(hex1.port_a1, building.port_a) annotation (Line(
      points={{-70,-6},{-70,36},{-65.6,36}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(hex1.port_b1, building.port_b) annotation (Line(
      points={{-50,-6},{-58,-6},{-58,36.1},{-62.7,36.1}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated1.port_a, spl.port_1) annotation (Line(
      points={{4,-46},{4,-46}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl.port_3, hex.port_b2) annotation (Line(
      points={{-6,-36},{-10,-36},{-10,-18},{-14,-18}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated3.port_b, spl.port_2) annotation (Line(
      points={{-24,-44},{-20,-44},{-20,-46},{-16,-46}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl1.port_1, pipe_Insulated3.port_a) annotation (Line(
      points={{-52,-46},{-48,-46},{-48,-44},{-44,-44}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl1.port_3, hex1.port_b2) annotation (Line(
      points={{-62,-36},{-66,-36},{-66,-18},{-70,-18}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl1.port_2, pipe_Insulated4.port_a) annotation (Line(
      points={{-72,-46},{-82,-46},{-82,-24},{-90,-24}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated4.port_b, spl2.port_1) annotation (Line(
      points={{-90,-4},{-90,10},{-54,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl2.port_2, pipe_Insulated5.port_a) annotation (Line(
      points={{-34,10},{-28,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated5.port_b, spl3.port_1) annotation (Line(
      points={{-8,10},{-8,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(hex.port_a2, spl3.port_3) annotation (Line(
      points={{6,-18},{4,-18},{4,0},{2,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(spl3.port_2, pipe_Insulated2.port_a) annotation (Line(
      points={{12,10},{20,10},{20,12},{24,12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(hex1.port_a2, spl2.port_3) annotation (Line(
      points={{-50,-18},{-44,-18},{-44,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(realExpression.y, boiler.TSet) annotation (Line(
      points={{69,58},{66,58},{66,33}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),      graphics));
end district3;
