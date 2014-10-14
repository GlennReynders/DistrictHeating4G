within DistrictHeating.Toplevel;
model district2

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
    annotation (Placement(transformation(extent={{-76,18},{-56,38}})));

  IDEAS.Fluid.Sources.FixedBoundary bou(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    use_T=false,
    nPorts=1)
    annotation (Placement(transformation(extent={{-64,-92},{-44,-72}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort Tsupply(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, m_flow_nominal=2)
    annotation (Placement(transformation(extent={{48,-24},{68,-4}})));
  IDEAS.Fluid.Movers.Pump pump(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m_flow_nominal=0.1,
    useInput=true)
    annotation (Placement(transformation(extent={{24,-24},{44,-4}})));
  IDEAS.Fluid.Production.Boiler boiler(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    QNom=10000,
    m_flow_nominal=0.1,
    modulationMin=25,
    modulationStart=30) annotation (Placement(transformation(
        extent={{10,11},{-10,-11}},
        rotation=90,
        origin={-2,1})));
  Modelica.Blocks.Sources.Constant const(k=273.15 + 60)
    annotation (Placement(transformation(extent={{-14,32},{6,52}})));
  IDEAS.Fluid.FixedResistances.Pipe_Insulated pipe_Insulated(
    UA=10,
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m=1000,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{6,-68},{26,-60}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=293.15)
    annotation (Placement(transformation(extent={{-20,-98},{0,-78}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort Treturn(
                                                redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, m_flow_nominal=2)
    annotation (Placement(transformation(extent={{-46,-26},{-26,-6}})));
  IDEAS.Controls.Continuous.LimPID conPID(controllerType=Modelica.Blocks.Types.SimpleController.P)
    annotation (Placement(transformation(extent={{46,30},{66,50}})));
  Modelica.Blocks.Sources.Constant returnset(k=273.15 + 50)
    annotation (Placement(transformation(extent={{12,52},{32,72}})));
  IDEAS.Fluid.FixedResistances.Pipe_Insulated pipe_Insulated1(
    dp_nominal=2,
    m=1,
    UA=1,
    m_flow_nominal=0.1,
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater)
    annotation (Placement(transformation(extent={{-26,-22},{-6,-14}})));
  IDEAS.Fluid.FixedResistances.Pipe_Insulated pipe_Insulated2(
    dp_nominal=2,
    m=1,
    UA=1,
    m_flow_nominal=0.1,
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater)
    annotation (Placement(transformation(extent={{2,-20},{22,-12}})));
equation
  connect(bou.ports[1], building.port_a) annotation (Line(
      points={{-44,-82},{-66,-82},{-66,18},{-67.6,18}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump.port_b, Tsupply.port_a) annotation (Line(
      points={{44,-14},{48,-14}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(const.y, boiler.TSet) annotation (Line(
      points={{7,42},{16,42},{16,3},{10,3}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pipe_Insulated.port_a, building.port_a) annotation (Line(
      points={{6,-64},{-70,-64},{-70,18},{-67.6,18}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fixedTemperature.port, pipe_Insulated.heatPort) annotation (Line(
      points={{0,-88},{8,-88},{8,-68},{16,-68}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(Tsupply.port_b, pipe_Insulated.port_b) annotation (Line(
      points={{68,-14},{92,-14},{92,-64},{26,-64}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building.port_b, Treturn.port_a) annotation (Line(
      points={{-64.7,18.1},{-54.35,18.1},{-54.35,-16},{-46,-16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(Treturn.T, conPID.u_m) annotation (Line(
      points={{-36,-5},{8,-5},{8,28},{56,28}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(returnset.y, conPID.u_s) annotation (Line(
      points={{33,62},{38,62},{38,40},{44,40}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(conPID.y, pump.m_flowSet) annotation (Line(
      points={{67,40},{74,40},{74,8},{34,8},{34,-3.6}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(Treturn.port_b, pipe_Insulated1.port_a) annotation (Line(
      points={{-26,-16},{-26,-18}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(boiler.port_a, pipe_Insulated1.port_b) annotation (Line(
      points={{-6,-8},{-6,-18}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated1.heatPort, fixedTemperature.port) annotation (Line(
      points={{-16,-22},{-8,-22},{-8,-88},{0,-88}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(boiler.port_b, pipe_Insulated2.port_a) annotation (Line(
      points={{2,-8},{2,-16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated2.port_b, pump.port_a) annotation (Line(
      points={{22,-16},{24,-16},{24,-14}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_Insulated2.heatPort, fixedTemperature.port) annotation (Line(
      points={{12,-20},{6,-20},{6,-88},{0,-88}},
      color={191,0,0},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),      graphics));
end district2;
