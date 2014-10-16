within DistrictHeating4G.HeatingSystem;
model Radiator
  extends DistrictHeating4G.HeatingSystem.Interfaces.PartialHydraulicHeating;

  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow[nEmbPorts] fixedHeatFlow(Q_flow=0)
    annotation (Placement(transformation(extent={{-152,22},{-162,32}})));
equation
  QHeaSys = -sum(emission.heatPortCon.Q_flow) - sum(emission.heatPortRad.Q_flow);
  P[1] = sum(pumpRad.PEl);
  Q[1] = 0;

  connect(port_supply, pipeSupply.port_a) annotation (Line(
      points={{-60,100},{-60,58},{-16,58}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(port_return, pipeReturn.port_b) annotation (Line(
      points={{-120,100},{-120,36},{-48,36},{-48,-92},{-18,-92}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fixedHeatFlow.port, heatPortEmb) annotation (Line(
      points={{-162,27},{-180,27},{-180,60},{-200,60}},
      color={191,0,0},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
            -100},{200,100}}), graphics));
end Radiator;
