within DistrictHeating4G.Toplevel;
model neighborhood_L

  inner IDEAS.SimInfoManager sim
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  DistrictHeating4G.Interfaces.Baseclasses.Building building1(
    redeclare CE.Occupants.ISO13790 occupant,
    redeclare IDEAS.Interfaces.BaseClasses.CausalInhomeFeeder inHomeGrid,
    redeclare CE.Models.SL1 building(orientation=0),
    redeclare IDEAS.VentilationSystems.None ventilationSystem,
    DH=true,
    redeclare DistrictHeating4G.HeatingSystem.Radiator heatingSystem)
                                      annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-90,-30})));
  DistrictHeating4G.Interfaces.Baseclasses.Building building3(
    redeclare CE.Occupants.ISO13790 occupant,
    redeclare IDEAS.Interfaces.BaseClasses.CausalInhomeFeeder inHomeGrid,
    redeclare CE.Models.T1 building(orientation=0),
    redeclare IDEAS.VentilationSystems.None ventilationSystem,
    redeclare DistrictHeating4G.HeatingSystem.Radiator heatingSystem,
    DH=true)                          annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-70,-30})));
  DistrictHeating4G.Interfaces.Baseclasses.Building building4(
    redeclare CE.Occupants.ISO13790 occupant,
    redeclare IDEAS.Interfaces.BaseClasses.CausalInhomeFeeder inHomeGrid,
    redeclare CE.Models.T2 building(orientation=0),
    redeclare IDEAS.VentilationSystems.None ventilationSystem,
    redeclare DistrictHeating4G.HeatingSystem.Radiator heatingSystem,
    DH=true)                          annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,-30})));
  DistrictHeating4G.Interfaces.Baseclasses.Building building5(
    redeclare CE.Occupants.ISO13790 occupant,
    redeclare IDEAS.Interfaces.BaseClasses.CausalInhomeFeeder inHomeGrid,
    redeclare CE.Models.T1 building(orientation=0),
    redeclare IDEAS.VentilationSystems.None ventilationSystem,
    redeclare DistrictHeating4G.HeatingSystem.Radiator heatingSystem,
    DH=true)                          annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,-30})));
  DistrictHeating4G.Interfaces.Baseclasses.Building building7(
    redeclare CE.Occupants.ISO13790 occupant,
    redeclare IDEAS.Interfaces.BaseClasses.CausalInhomeFeeder inHomeGrid,
    redeclare CE.Models.SR2 building(orientation=0),
    redeclare IDEAS.VentilationSystems.None ventilationSystem,
    redeclare DistrictHeating4G.HeatingSystem.Radiator heatingSystem,
    DH=true)                          annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-10,-30})));
  DistrictHeating4G.Interfaces.Baseclasses.Building building17(
    redeclare CE.Models.SL1 building(orientation=-1.5707963267949),
    redeclare CE.Occupants.ISO13790 occupant,
    redeclare IDEAS.Interfaces.BaseClasses.CausalInhomeFeeder inHomeGrid,
    redeclare IDEAS.VentilationSystems.None ventilationSystem,
    redeclare DistrictHeating4G.HeatingSystem.Radiator heatingSystem,
    DH=true)
    annotation (Placement(transformation(extent={{-20,0},{0,20}})));
  DistrictHeating4G.Interfaces.Baseclasses.Building building19(
    redeclare CE.Models.T1 building(orientation=-1.5707963267949),
    redeclare CE.Occupants.ISO13790 occupant,
    redeclare IDEAS.Interfaces.BaseClasses.CausalInhomeFeeder inHomeGrid,
    redeclare IDEAS.VentilationSystems.None ventilationSystem,
    redeclare DistrictHeating4G.HeatingSystem.Radiator heatingSystem,
    DH=true)
    annotation (Placement(transformation(extent={{-20,20},{0,40}})));
  DistrictHeating4G.Interfaces.Baseclasses.Building building21(
    redeclare CE.Models.T2 building(orientation=-1.5707963267949),
    redeclare CE.Occupants.ISO13790 occupant,
    redeclare IDEAS.Interfaces.BaseClasses.CausalInhomeFeeder inHomeGrid,
    redeclare IDEAS.VentilationSystems.None ventilationSystem,
    redeclare DistrictHeating4G.HeatingSystem.Radiator heatingSystem,
    DH=true)
    annotation (Placement(transformation(extent={{-20,40},{0,60}})));
  DistrictHeating4G.Interfaces.Baseclasses.Building building23(
    redeclare CE.Models.T2 building(orientation=-1.5707963267949),
    redeclare CE.Occupants.ISO13790 occupant,
    redeclare IDEAS.Interfaces.BaseClasses.CausalInhomeFeeder inHomeGrid,
    redeclare IDEAS.VentilationSystems.None ventilationSystem,
    redeclare DistrictHeating4G.HeatingSystem.Radiator heatingSystem,
    DH=true)
    annotation (Placement(transformation(extent={{-20,60},{0,80}})));
  inner Modelica.Fluid.System system
    annotation (Placement(transformation(extent={{-80,80},{-60,100}})));
  Substations.HXWithBypass hXWithBypass(staticFourPortHeatMassExchanger(
        m2_flow_nominal=1), pump(m_flow_nominal=1))
    annotation (Placement(transformation(extent={{-94,-50},{-86,-42}})));
  Substations.HXWithBypass hXWithBypass1
    annotation (Placement(transformation(extent={{-74,-50},{-66,-42}})));
  Substations.HXWithBypass hXWithBypass2
    annotation (Placement(transformation(extent={{-54,-50},{-46,-42}})));
  Substations.HXWithBypass hXWithBypass3
    annotation (Placement(transformation(extent={{-34,-50},{-26,-42}})));
  Substations.HXWithBypass hXWithBypass4
    annotation (Placement(transformation(extent={{-14,-50},{-6,-42}})));
  Production.BoilerWithPump boilerWithPump(boiler(
      QNom=50000,
      mWater=1000,
      m_flow_nominal=1), pump(m_flow_nominal=1))
    annotation (Placement(transformation(extent={{22,-42},{42,-22}})));
  Substations.HXWithBypass hXWithBypass5
    annotation (Placement(transformation(extent={{-4,-4},{4,4}},
        rotation=90,
        origin={8,-8})));
  Substations.HXWithBypass hXWithBypass6
    annotation (Placement(transformation(extent={{-4,-4},{4,4}},
        rotation=90,
        origin={8,18})));
  Substations.HXWithBypass hXWithBypass7
    annotation (Placement(transformation(extent={{-4,-4},{4,4}},
        rotation=90,
        origin={8,42})));
  Substations.HXWithBypass hXWithBypass8
    annotation (Placement(transformation(extent={{-4,-4},{4,4}},
        rotation=90,
        origin={8,62})));
  IDEAS.Fluid.Sources.FixedBoundary bou(nPorts=1, redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater)
    annotation (Placement(transformation(extent={{66,-40},{50,-24}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=286)
    annotation (Placement(transformation(extent={{38,36},{58,56}})));
equation
  connect(building1.port_return, hXWithBypass.flowPort_b1) annotation (Line(
      points={{-91.6,-40},{-92,-40},{-92,-42},{-90.8,-42}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building1.port_supply, hXWithBypass.flowPort_a1) annotation (Line(
      points={{-88.7,-39.9},{-88,-39.9},{-88,-42},{-89.2,-42}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(hXWithBypass.flowPort_supply_in, hXWithBypass1.flowPort_supply_out)
    annotation (Line(
      points={{-86,-45.2},{-80,-45.2},{-80,-45.2},{-74,-45.2}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass1.flowPort_supply_in, hXWithBypass2.flowPort_supply_out)
    annotation (Line(
      points={{-66,-45.2},{-60,-45.2},{-60,-45.2},{-54,-45.2}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass2.flowPort_supply_in, hXWithBypass3.flowPort_supply_out)
    annotation (Line(
      points={{-46,-45.2},{-40,-45.2},{-40,-45.2},{-34,-45.2}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass3.flowPort_supply_in, hXWithBypass4.flowPort_supply_out)
    annotation (Line(
      points={{-26,-45.2},{-14,-45.2}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(building17.port_supply, hXWithBypass5.flowPort_a1) annotation (Line(
      points={{-8.7,0.1},{-2.35,0.1},{-2.35,-7.2},{4,-7.2}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building17.port_return, hXWithBypass5.flowPort_b1) annotation (Line(
      points={{-11.6,0},{-4,0},{-4,-8.8},{4,-8.8}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building19.port_supply, hXWithBypass6.flowPort_a1) annotation (Line(
      points={{-8.7,20.1},{-2.35,20.1},{-2.35,18.8},{4,18.8}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building19.port_return, hXWithBypass6.flowPort_b1) annotation (Line(
      points={{-11.6,20},{-4,20},{-4,17.2},{4,17.2}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building21.port_supply, hXWithBypass7.flowPort_a1) annotation (Line(
      points={{-8.7,40.1},{-3.35,40.1},{-3.35,42.8},{4,42.8}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building21.port_return, hXWithBypass7.flowPort_b1) annotation (Line(
      points={{-11.6,40},{-4,40},{-4,41.2},{4,41.2}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building23.port_supply, hXWithBypass8.flowPort_a1) annotation (Line(
      points={{-8.7,60.1},{-2.35,60.1},{-2.35,62.8},{4,62.8}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building23.port_return, hXWithBypass8.flowPort_b1) annotation (Line(
      points={{-11.6,60},{-4,60},{-4,61.2},{4,61.2}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(hXWithBypass4.flowPort_supply_in, hXWithBypass5.flowPort_supply_out)
    annotation (Line(
      points={{-6,-45.2},{6,-45.2},{6,-12},{7.2,-12}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass5.flowPort_supply_in, hXWithBypass6.flowPort_supply_out)
    annotation (Line(
      points={{7.2,-4},{7.2,-4},{7.2,14}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass6.flowPort_supply_in, hXWithBypass7.flowPort_supply_out)
    annotation (Line(
      points={{7.2,22},{7.2,38}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass7.flowPort_supply_in, hXWithBypass8.flowPort_supply_out)
    annotation (Line(
      points={{7.2,46},{7.2,46},{7.2,58}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass8.flowPort_supply_in, hXWithBypass8.flowPort_return_out)
    annotation (Line(
      points={{7.2,66},{8.8,66}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass8.flowPort_return_in, hXWithBypass7.flowPort_return_out)
    annotation (Line(
      points={{8.8,58},{8.8,58},{8.8,46}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass7.flowPort_return_in, hXWithBypass6.flowPort_return_out)
    annotation (Line(
      points={{8.8,38},{8.8,38},{8.8,22}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass6.flowPort_return_in, hXWithBypass5.flowPort_return_out)
    annotation (Line(
      points={{8.8,14},{8.8,14},{8.8,-4}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass5.flowPort_return_in, boilerWithPump.flowPort_return)
    annotation (Line(
      points={{8.8,-12},{10,-12},{10,-32},{22,-32}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(boilerWithPump.flowPort_supply, hXWithBypass4.flowPort_return_out)
    annotation (Line(
      points={{42,-32},{42,-46.8},{-6,-46.8}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass4.flowPort_return_in, hXWithBypass3.flowPort_return_out)
    annotation (Line(
      points={{-14,-46.8},{-20,-46.8},{-20,-46.8},{-26,-46.8}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass3.flowPort_return_in, hXWithBypass2.flowPort_return_out)
    annotation (Line(
      points={{-34,-46.8},{-40,-46.8},{-40,-46.8},{-46,-46.8}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass2.flowPort_return_in, hXWithBypass1.flowPort_return_out)
    annotation (Line(
      points={{-54,-46.8},{-60,-46.8},{-60,-46.8},{-66,-46.8}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass1.flowPort_return_in, hXWithBypass.flowPort_return_out)
    annotation (Line(
      points={{-74,-46.8},{-80,-46.8},{-80,-46.8},{-86,-46.8}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass.flowPort_return_in, hXWithBypass.flowPort_supply_out)
    annotation (Line(
      points={{-94,-46.8},{-94,-45.2}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(bou.ports[1], boilerWithPump.flowPort_supply) annotation (Line(
      points={{50,-32},{42,-32}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(hXWithBypass1.flowPort_b1, building3.port_return) annotation (Line(
      points={{-70.8,-42},{-71.6,-42},{-71.6,-40}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass1.flowPort_a1, building3.port_supply) annotation (Line(
      points={{-69.2,-42},{-68.7,-42},{-68.7,-39.9}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass2.flowPort_b1, building4.port_return) annotation (Line(
      points={{-50.8,-42},{-51.6,-42},{-51.6,-40}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass2.flowPort_a1, building4.port_supply) annotation (Line(
      points={{-49.2,-42},{-48.7,-42},{-48.7,-39.9}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass3.flowPort_b1, building5.port_return) annotation (Line(
      points={{-30.8,-42},{-31.6,-42},{-31.6,-40}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass3.flowPort_a1, building5.port_supply) annotation (Line(
      points={{-29.2,-42},{-28.7,-42},{-28.7,-39.9}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass4.flowPort_b1, building7.port_return) annotation (Line(
      points={{-10.8,-42},{-11.6,-42},{-11.6,-40}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(hXWithBypass4.flowPort_a1, building7.port_supply) annotation (Line(
      points={{-9.2,-42},{-8.7,-42},{-8.7,-39.9}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(realExpression.y, hXWithBypass8.u) annotation (Line(
      points={{59,46},{36,46},{36,62},{12.24,62}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(realExpression.y, hXWithBypass7.u) annotation (Line(
      points={{59,46},{36,46},{36,42},{12.24,42}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(realExpression.y, hXWithBypass6.u) annotation (Line(
      points={{59,46},{36,46},{36,18},{12.24,18}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(realExpression.y, hXWithBypass5.u) annotation (Line(
      points={{59,46},{36,46},{36,-8},{12.24,-8}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(realExpression.y, hXWithBypass4.u) annotation (Line(
      points={{59,46},{98,46},{98,-50.24},{-10,-50.24}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(realExpression.y, hXWithBypass3.u) annotation (Line(
      points={{59,46},{98,46},{98,-50.24},{-30,-50.24}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(realExpression.y, hXWithBypass2.u) annotation (Line(
      points={{59,46},{4,46},{4,-50.24},{-50,-50.24}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(realExpression.y, hXWithBypass1.u) annotation (Line(
      points={{59,46},{98,46},{98,-50.24},{-70,-50.24}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(realExpression.y, hXWithBypass.u) annotation (Line(
      points={{59,46},{98,46},{98,-50.24},{-90,-50.24}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(extent={{-100,-100},{120,100}},
          preserveAspectRatio=false), graphics={
        Line(
          points={{-100,-48},{120,-48}},
          color={0,128,255},
          smooth=Smooth.None,
          thickness=0.5),
        Line(
          points={{6,80},{6,-48}},
          color={0,128,255},
          smooth=Smooth.None,
          thickness=0.5),
        Line(
          points={{-100,-46},{4,-46}},
          smooth=Smooth.None,
          color={127,0,0}),
        Line(
          points={{-100,-56},{120,-56}},
          smooth=Smooth.None,
          color={127,0,0}),
        Line(
          points={{14,80},{14,-46}},
          color={127,0,0},
          smooth=Smooth.None),
        Line(
          points={{4,80},{4,-46}},
          color={127,0,0},
          smooth=Smooth.None),
        Line(
          points={{14,-46},{120,-46}},
          color={127,0,0},
          smooth=Smooth.None),
        Ellipse(
          extent={{-106,-45},{-100,-51}},
          lineColor={0,128,255},
          lineThickness=0.5),
        Ellipse(
          extent={{-110,-45},{-104,-51}},
          lineColor={0,128,255},
          lineThickness=0.5)}), Icon(coordinateSystem(extent={{-100,-100},{120,
            100}})),
    experiment(
      StopTime=3.1536e+007,
      Interval=3600,
      __Dymola_Algorithm="Lsodar"),
    __Dymola_experimentSetupOutput);
end neighborhood_L;
