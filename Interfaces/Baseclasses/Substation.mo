within DistrictHeating4G.Interfaces.Baseclasses;
partial model Substation "Interface for a local substation"

  IDEAS.Fluid.Interfaces.FlowPort_b flowPort_b
    annotation (Placement(transformation(extent={{-50,88},{-30,108}})));
  IDEAS.Fluid.Interfaces.FlowPort_a flowPort_a
    annotation (Placement(transformation(extent={{30,88},{50,108}})));
  IDEAS.Fluid.Interfaces.FlowPort_b flowPort_b1
    annotation (Placement(transformation(extent={{30,-108},{50,-88}})));
  IDEAS.Fluid.Interfaces.FlowPort_a flowPort_a1
    annotation (Placement(transformation(extent={{-50,-108},{-30,-88}})));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(graphics={
        Line(
          points={{40,-68},{40,-94}},
          color={0,0,255},
          smooth=Smooth.None),
        Line(
          points={{-40,-68},{-40,-92}},
          color={0,0,255},
          smooth=Smooth.None),
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
          points={{-40,62},{-40,98}},
          color={0,0,255},
          smooth=Smooth.None),
        Line(
          points={{40,62},{40,98}},
          color={0,0,255},
          smooth=Smooth.None)}));
end Substation;
