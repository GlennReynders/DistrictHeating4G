within DistrictHeating.Toplevel;
model district

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
        DH=true, hex(
        m1_flow_nominal=2,
        m2_flow_nominal=2,
        dp1_nominal=200,
        dp2_nominal=200)))
    annotation (Placement(transformation(extent={{-44,-10},{-24,10}})));

  IDEAS.Fluid.FixedResistances.Pipe_HeatPort pipe_HeatPort(
    redeclare package Medium = Modelica.Media.Examples.TwoPhaseWater,
    m_flow_nominal=3,
    dp_nominal=10,
    m=1000) annotation (Placement(transformation(extent={{0,-24},{20,-4}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow fixedHeatFlow(Q_flow=
        5000)
    annotation (Placement(transformation(extent={{-42,26},{-22,46}})));
  IDEAS.Fluid.Sources.FixedBoundary bou(
    redeclare package Medium = Modelica.Media.Examples.TwoPhaseWater,
    use_T=false,
    nPorts=1)
    annotation (Placement(transformation(extent={{-64,-92},{-44,-72}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium =
        Modelica.Media.Examples.TwoPhaseWater, m_flow_nominal=2)
    annotation (Placement(transformation(extent={{48,-24},{68,-4}})));
  IDEAS.Controls.Continuous.LimPID conPID(controllerType=Modelica.Blocks.Types.SimpleController.P)
    annotation (Placement(transformation(extent={{40,28},{60,48}})));
  Modelica.Blocks.Sources.Constant const(k=273.15 + 80)
    annotation (Placement(transformation(extent={{-4,44},{16,64}})));
  IDEAS.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium = Modelica.Media.Examples.TwoPhaseWater,
    m_flow_nominal=2,
    nPorts=1,
    V=100) annotation (Placement(transformation(extent={{26,-74},{46,-54}})));
  IDEAS.Fluid.Movers.Pump pump(redeclare package Medium =
        Modelica.Media.Examples.TwoPhaseWater, m_flow_nominal=2)
    annotation (Placement(transformation(extent={{24,-24},{44,-4}})));
equation
  connect(fixedHeatFlow.port, pipe_HeatPort.heatPort) annotation (Line(
      points={{-22,36},{-6,36},{-6,-4},{10,-4}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(conPID.u_m, senTem.T) annotation (Line(
      points={{50,26},{54,26},{54,-3},{58,-3}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(const.y, conPID.u_s) annotation (Line(
      points={{17,54},{26,54},{26,38},{38,38}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(vol.ports[1], pipe_HeatPort.port_b) annotation (Line(
      points={{36,-74},{20,-74},{20,-14}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building.port_b, pipe_HeatPort.port_a) annotation (Line(
      points={{-32.7,-9.9},{-16.35,-9.9},{-16.35,-14},{0,-14}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senTem.port_b, building.port_a) annotation (Line(
      points={{68,-14},{78,-14},{78,-48},{-36,-48},{-36,-10},{-35.6,-10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou.ports[1], building.port_a) annotation (Line(
      points={{-44,-82},{-42,-82},{-42,-10},{-35.6,-10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_HeatPort.port_b, pump.port_a) annotation (Line(
      points={{20,-14},{24,-14}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump.port_b, senTem.port_a) annotation (Line(
      points={{44,-14},{48,-14}},
      color={0,127,255},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
            -100,-100},{100,100}}), graphics));
end district;
