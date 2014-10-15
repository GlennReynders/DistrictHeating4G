within DistrictHeating4G.Production;
model BoilerWithPump "Boiler for production of hot water"

  //Extensions
  extends DistrictHeating4G.Interfaces.Baseclasses.Production(flowPort_supply(
        redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater), flowPort_return(
        redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater));

  IDEAS.Fluid.Production.Boiler boiler(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    QNom=10000,
    mWater=50,
    m_flow_nominal=0.1) annotation (Placement(transformation(
        extent={{-10,-11},{10,11}},
        rotation=270,
        origin={-2,27})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=350)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={32,28})));
  IDEAS.Fluid.Movers.Pump pump(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, m_flow_nominal=0.2)
    annotation (Placement(transformation(extent={{-48,-10},{-28,10}})));
equation
  connect(realExpression.y,boiler. TSet) annotation (Line(
      points={{21,28},{10,28},{10,27}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(boiler.port_b, flowPort_supply) annotation (Line(
      points={{2,16},{2,0},{100,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(flowPort_return, pump.port_a) annotation (Line(
      points={{-100,0},{-48,0}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(boiler.port_a, pump.port_b) annotation (Line(
      points={{-6,16},{-18,16},{-18,0},{-28,0}},
      color={0,127,255},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics));
end BoilerWithPump;
