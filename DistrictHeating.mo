within ;
package DistrictHeating
  model heatexchangerDH
    "district heating sub station buildings constant efficiency"
    extends IDEAS.HeatingSystems.Interfaces.Partial_IdealHeating;
    extends IDEAS.Interfaces.BaseClasses.HeatingSystem(
      final isHea = true,
      final isCoo = false,
      final nConvPorts = nZones,
      final nRadPorts = nZones,
      final nTemSen = nZones,
      final nEmbPorts=0);
       parameter Real eff= 0.9 "efficiency heat exchanger";
    IDEAS.Fluid.FixedResistances.Pipe_HeatPort pipe_HeatPort(
      m_flow_nominal=10,
      dp_nominal=2,
      redeclare package Medium = Modelica.Media.Examples.TwoPhaseWater)
      annotation (Placement(transformation(extent={{-76,8},{-56,28}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prescribedHeatFlow
      annotation (Placement(transformation(extent={{-60,54},{-40,74}})));
  equation
     for i in 1:nZones loop
       if noEvent((TSet[i] - TSensor[i]) > 0) then
         QHeatZone[i] = IDEAS.Utilities.Math.Functions.smoothMin(x1=C[i]*(TSet[i] - TSensor[i])/t, x2=QNom[i],deltaX=1);
       else
         QHeatZone[i] = 0;
       end if;
       heatPortRad[i].Q_flow = -fractionRad[i]*QHeatZone[i];
       heatPortCon[i].Q_flow = -(1 - fractionRad[i])*QHeatZone[i];
     end for;
    QHeaSys = sum(QHeatZone);
     prescribedHeatFlow.Q_flow   =-sum(QHeatZone)/eff;
    P[1] = 0;
    Q[1] = 0;
    connect(port_b, pipe_HeatPort.port_a) annotation (Line(
        points={{-120,100},{-98,100},{-98,18},{-76,18}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pipe_HeatPort.port_b, port_a) annotation (Line(
        points={{-56,18},{-78,18},{-78,100},{-100,100}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(prescribedHeatFlow.port, pipe_HeatPort.heatPort) annotation (Line(
        points={{-40,64},{-52,64},{-52,28},{-66,28}},
        color={191,0,0},
        smooth=Smooth.None));
    annotation (Documentation(info="<html>
<p><b>Description</b> </p>
<p>Ideal heating (no hydraulics) but with limited power <i>QNom</i> per zone. There are no radiators. This model assumes a thermal inertia of each zone and computes the heat flux that would be required to heat up the zone to the set point within a time <i>t</i>. This heat flux is limited to <i>QNom</i> and splitted in a radiative and a convective part which are imposed on the heatPorts <i>heatPortRad</i> and <i>heatPortCon</i> respectively. A COP can be passed in order to compute the electricity consumption of the heating.</p>
<p><u>Note</u>: the responsiveness of the system is influenced by the time constant <i>t</i>.  For small values of<i> t</i>, this system is close to ideal, but for larger values, there may still be deviations between the zone temperature and it&apos;s set point. </p>
<p><h4>Assumptions and limitations </h4></p>
<p><ol>
<li>No inertia; responsiveness modelled by time constant <i>t</i> for reaching the temperature set point. </li>
<li>Limited output power according to <i>QNom[nZones]</i></li>
<li>Heat emitted through <i>heatPortRad</i> and <i>heatPortCon</i> </li>
</ol></p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Connect the heating system to the corresponding heatPorts of a <a href=\"modelica://IDEAS.Interfaces.BaseClasses.Structure\">structure</a>. </li>
<li>Connect <i>TSet</i> and <i>TSensor</i> </li>
<li>Connect <i>plugLoad </i>to an inhome grid. A<a href=\"modelica://IDEAS.Interfaces.BaseClasses.CausalInhomeFeeder\"> dummy inhome grid like this</a> has to be used if no inhome grid is to be modelled. </li>
<li>Set all parameters that are required. </li>
</ol></p>
<p><h4>Validation </h4></p>
<p>No validation performed.</p>
<p><h4>Example </h4></p>
<p>An example of the use of this model can be found in<a href=\"modelica://IDEAS.Thermal.HeatingSystems.Examples.IdealRadiatorHeating\"> IDEAS.Thermal.HeatingSystems.Examples.IdealRadiatorHeating</a>.</p>
</html>",   revisions="<html>
<p><ul>
<li>2013 June, Roel De Coninck: reworking interface and documentation</li>
<li>2011, Roel De Coninck: first version</li>
</ul></p>
</html>"),   Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},
              {200,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
          graphics));
  end heatexchangerDH;

  model district

    inner IDEAS.SimInfoManager sim(occBeh=false, DHW=false)
      annotation (Placement(transformation(extent={{-98,70},{-78,90}})));
    inner Modelica.Fluid.System system
      annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
    IDEAS.Interfaces.Building building(
      redeclare IDEAS.HeatingSystems.Examples.DummyBuilding building(nZones=1,
          nEmb=0),
      redeclare IDEAS.VentilationSystems.None ventilationSystem,
      redeclare IDEAS.Occupants.Standards.ISO13790 occupant,
      DH=true,
      redeclare DistrictHeating.heatexchangerDH heatingSystem(DH=true))
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
      annotation (Placement(transformation(extent={{-66,-42},{-46,-22}})));
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
        points={{-46,-32},{-42,-32},{-42,-10},{-35.6,-10}},
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
  annotation (uses(IDEAS(version="0.1"), Modelica(version="3.2.1")));
end DistrictHeating;
