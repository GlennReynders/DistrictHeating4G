within DistrictHeating4G.Interfaces;
partial model Network "Interface for a district heating network"
  Baseclasses.Building building(DH=true)
    annotation (Placement(transformation(extent={{-40,14},{-20,34}})));
  replaceable Baseclasses.Substation substation
    annotation (Placement(transformation(extent={{-40,-20},{-20,0}})));
  replaceable Baseclasses.Production production annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,-8})));
  replaceable Baseclasses.Substation substation1
    annotation (Placement(transformation(extent={{-76,-20},{-56,0}})));
  Baseclasses.Building building1(DH=true)
    annotation (Placement(transformation(extent={{-76,14},{-56,34}})));
equation
  connect(building.port_a, substation.flowPort_b1) annotation (Line(
      points={{-31.6,14},{-32,14},{-32,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building.port_b, substation.flowPort_a1) annotation (Line(
      points={{-28.7,14.1},{-28.7,14},{-28,14},{-28,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(substation.flowPort_b2, production.flowPort_a) annotation (Line(
      points={{-20,-8},{4,-8},{4,8},{40,8},{40,2}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(substation.flowPort_a2, production.flowPort_b) annotation (Line(
      points={{-20,-12},{4,-12},{4,-24},{40,-24},{40,-18}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(substation.flowPort_b, substation1.flowPort_a2) annotation (Line(
      points={{-40,-12},{-56,-12}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(substation1.flowPort_b1, building1.port_a) annotation (Line(
      points={{-68,0},{-68,14},{-67.6,14}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(substation1.flowPort_a1, building1.port_b) annotation (Line(
      points={{-64,0},{-64,8},{-64,14.1},{-64.7,14.1}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(substation1.flowPort_a, substation1.flowPort_b) annotation (Line(
      points={{-76,-8},{-86,-8},{-86,-12},{-76,-12}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(substation1.flowPort_supply_b, substation.flowPort_supply_a)
    annotation (Line(
      points={{-56,-8},{-40,-8}},
      color={0,0,0},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(
          preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics
        ={
        Line(
          points={{-60,-40},{-40,-40},{-40,0},{40,0},{40,40},{80,40}},
          color={0,0,255},
          smooth=Smooth.None),
        Line(
          points={{0,0},{0,-40},{40,-40}},
          color={0,0,255},
          smooth=Smooth.None),
        Line(
          points={{-12,0},{-12,80}},
          color={0,0,255},
          smooth=Smooth.None),
        Line(
          points={{-12,40},{-60,40}},
          color={0,0,255},
          smooth=Smooth.None)}));
end Network;
