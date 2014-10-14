within DistrictHeating4G.Interfaces;
partial model Network "Interface for a district heating network"
  Baseclasses.Building building(DH=true)
    annotation (Placement(transformation(extent={{-42,14},{-22,34}})));
  replaceable Baseclasses.Substation substation
    annotation (Placement(transformation(extent={{-40,-20},{-20,0}})));
  replaceable Baseclasses.Production production
    annotation (Placement(transformation(extent={{14,-40},{34,-20}})));
equation
  connect(building.port_b, substation.flowPort_a) annotation (Line(
      points={{-30.7,14.1},{-30.7,7.05},{-26,7.05},{-26,-0.2}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(building.port_a, substation.flowPort_b) annotation (Line(
      points={{-33.6,14},{-34,14},{-34,-0.2}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(substation.flowPort_b1, production.flowPort_a) annotation (Line(
      points={{-26,-19.8},{-26,-19.8},{-26,-30},{14,-30}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(substation.flowPort_a1, production.flowPort_b) annotation (Line(
      points={{-34,-19.8},{-34,-60},{46,-60},{46,-30},{34,-30}},
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
