within ;
package DistrictHeating

  package Toplevel "district models"
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
        redeclare DistrictHeating.HeatingSystem.heatexchangerDH heatingSystem(
            DH=true))
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
  end Toplevel;

  package HeatingSystem
    model heatexchangerDH
      "district heating sub station buildings constant efficiency"
      import DistrictHeating;
      extends IDEAS.HeatingSystems.Interfaces.Partial_IdealHeating;
      extends DistrictHeating.Interfaces.HeatingSystem(
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
</html>",     revisions="<html>
<p><ul>
<li>2013 June, Roel De Coninck: reworking interface and documentation</li>
<li>2011, Roel De Coninck: first version</li>
</ul></p>
</html>"),     Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},
                {200,100}}), graphics),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
            graphics));
    end heatexchangerDH;

    model Radiator
      extends Interfaces.PartialDistrictHeating;

    equation
      QHeaSys = -sum(emission.heatPortCon.Q_flow) - sum(emission.heatPortRad.Q_flow);
      P[1] = sum(pumpRad.PEl);
      Q[1] = 0;

    end Radiator;

    package Interfaces
      partial model PartialDistrictHeating "Hydraulic multi-zone heating "
        import DistrictHeating;
        replaceable package Medium =
            Modelica.Media.Water.ConstantPropertyLiquidWater;
        extends DistrictHeating.Interfaces.HeatingSystem(
          isHea = true,
          isCoo = false,
          nConvPorts = nZones,
          nRadPorts = nZones,
          nTemSen = nZones,
          nEmbPorts=0,
          nLoads=1,
          nZones=1,
          port_b(redeclare package Medium = Medium),
          port_a(redeclare package Medium = Medium));
        // --- Paramter: General parameters for the design (nominal) conditions and heat curve
        parameter Modelica.SIunits.Power[nZones] QNom(each min=0) = ones(nZones)*5000
          "Nominal power, can be seen as the max power of the emission system per zone";
        parameter Boolean minSup=true
          "true to limit the supply temperature on the lower side";
          parameter SI.Temperature TSupMin=273.15 + 30
          "Minimum supply temperature if enabled";
        parameter Modelica.SIunits.Temperature TSupNom=273.15 + 45
          "Nominal supply temperature";
        parameter Modelica.SIunits.TemperatureDifference dTSupRetNom=10
          "Nominal DT in the heating system";
        parameter Modelica.SIunits.Temperature[nZones] TRoomNom={294.15 for i in 1:
            nZones} "Nominal room temperature";
        parameter Modelica.SIunits.TemperatureDifference corFac_val = 0
          "correction term for TSet of the heating curve";
        parameter Modelica.SIunits.Time timeFilter=43200
          "Time constant for the filter of ambient temperature for computation of heating curve";
        final parameter Modelica.SIunits.MassFlowRate[nZones] m_flow_nominal = QNom/(4180.6*dTSupRetNom)
          "Nominal mass flow rates";
        // --- production components of hydraulic circuit
        // --- distribution components of hydraulic circuit
        IDEAS.Fluid.Movers.Pump[nZones] pumpRad(
          each useInput=true,
          each m=1,
          m_flow_nominal=m_flow_nominal,
          redeclare each package Medium = Medium,
          filteredMassFlowRate=true,
          riseTime=60)
                    annotation (Placement(transformation(extent={{88,64},{112,40}})));
        IDEAS.Fluid.Valves.Thermostatic3WayValve    idealCtrlMixer(m_flow_nominal=sum(
              m_flow_nominal), redeclare package Medium = Medium)
          annotation (Placement(transformation(extent={{34,46},{56,70}})));
        IDEAS.Fluid.FixedResistances.Pipe_Insulated pipeReturn(
          redeclare package Medium = Medium,
          m=1,
          UA=10,
          m_flow_nominal=sum(m_flow_nominal))
                 annotation (Placement(transformation(extent={{2,-88},{-18,-96}})));
        IDEAS.Fluid.FixedResistances.Pipe_Insulated pipeSupply(
          redeclare package Medium = Medium,
          m=1,
          UA=10,
          m_flow_nominal=sum(m_flow_nominal))
                 annotation (Placement(transformation(extent={{-16,54},{4,62}})));
        IDEAS.Fluid.FixedResistances.Pipe_Insulated[nZones] pipeReturnEmission(
          redeclare each package Medium = Medium,
          each m=1,
          each UA=10,
          m_flow_nominal=m_flow_nominal)
          annotation (Placement(transformation(extent={{148,-88},{128,-96}})));
        // --- emission components of hydraulic circuit
        replaceable IDEAS.Fluid.HeatExchangers.Radiators.Radiator[
                                                      nZones] emission(
            each TInNom=TSupNom,
            each TOutNom=TSupNom - dTSupRetNom,
            TZoneNom=TRoomNom,
            QNom=QNom,
            each powerFactor=3.37,
          redeclare each package Medium = Medium) constrainedby
          Fluid.HeatExchangers.Interfaces.EmissionTwoPort
          annotation (Placement(transformation(extent={{120,24},{150,44}})));
        // --- boudaries
        Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=293.15)
          "fixed temperature to simulate heat losses of hydraulic components"
          annotation (Placement(transformation(
              extent={{7,-7},{-7,7}},
              rotation=-90,
              origin={-127,-21})));
        IDEAS.Fluid.Sources.FixedBoundary absolutePressure(redeclare package
            Medium =
              Medium, use_T=false,
          nPorts=1)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-106,-114})));
        // --- controllers
        replaceable IDEAS.Controls.ControlHeating.Ctrl_Heating ctrl_Heating(
          heatingCurve(timeFilter=timeFilter),
          TSupNom=TSupNom,
          dTSupRetNom=dTSupRetNom,
          TSupMin=TSupMin,
          minSup=minSup,
          corFac_val=corFac_val,
          THeaterSet(start=293.15)) constrainedby
          Controls.ControlHeating.Interfaces.Partial_Ctrl_Heating(
          heatingCurve(timeFilter=timeFilter),
          TSupNom=TSupNom,
          dTSupRetNom=dTSupRetNom)
          "Controller for the heater and the emission set point "
          annotation (Placement(transformation(extent={{-160,54},{-140,74}})));
        replaceable IDEAS.Controls.Control_fixme.Hyst_NoEvent_Var[
                                                      nZones] heatingControl(each uLow_val=
              22, each uHigh_val=20)
          "onoff controller for the pumps of the emission circuits"
          annotation (Placement(transformation(extent={{-140,-80},{-120,-60}})));
        Modelica.Blocks.Sources.RealExpression THigh_val[nZones](y=0.5*ones(nZones))
          "Higher boudary for set point temperature"
          annotation (Placement(transformation(extent={{-174,-62},{-162,-42}})));
        Modelica.Blocks.Sources.RealExpression TLow_val[nZones](y=-0.5*ones(nZones))
          "Lower boundary for set point temperature"
          annotation (Placement(transformation(extent={{-174,-102},{-160,-82}})));
        Modelica.Blocks.Sources.RealExpression TSet_max(y=max(TSet))
          "maximum value of set point temperature" annotation (Placement(
              transformation(
              extent={{-12,-9},{12,9}},
              rotation=90,
              origin={-169,46})));
        Modelica.Blocks.Math.Add add[nZones](each k1=-1, each k2=+1)
          annotation (Placement(transformation(extent={{-174,-78},{-160,-64}})));
        // --- Interface
        Modelica.Blocks.Interfaces.RealInput TSet[nZones](    final quantity="ThermodynamicTemperature",unit="K",displayUnit="degC")
          "Set point temperature for the zones" annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=90,
              origin={-190,-108}),
                                iconTransformation(
              extent={{-14,-14},{14,14}},
              rotation=90,
              origin={-2,-104})));
        // --- Sensors
        IDEAS.Fluid.Sensors.TemperatureTwoPort senTemEm_in(redeclare package
            Medium =
              Medium, m_flow_nominal=sum(m_flow_nominal))
          "Inlet temperature of the emission system"
          annotation (Placement(transformation(extent={{62,42},{82,62}})));
        IDEAS.Fluid.Sensors.TemperatureTwoPort senTemHea_out(redeclare package
            Medium =
              Medium, m_flow_nominal=sum(m_flow_nominal))
          "Outlet temperature of the heater"
          annotation (Placement(transformation(extent={{-62,48},{-42,68}})));
        IDEAS.Fluid.Sensors.TemperatureTwoPort senTemEm_out(redeclare package
            Medium =
              Medium, m_flow_nominal=sum(m_flow_nominal))
          "Outlet temperature of the emission system" annotation (Placement(
              transformation(
              extent={{8,-8},{-8,8}},
              rotation=0,
              origin={90,-92})));
        IDEAS.Fluid.FixedResistances.SplitterFixedResistanceDpM spl(
          redeclare package Medium = Medium,
          m_flow_nominal={sum(m_flow_nominal),sum(m_flow_nominal),-sum(m_flow_nominal)},
          dp_nominal={0,0,0})
          annotation (Placement(transformation(extent={{76,-88},{68,-96}})));

        IDEAS.Fluid.MixingVolumes.MixingVolume vol(
          redeclare package Medium = Medium,
          m_flow_nominal=sum(m_flow_nominal),
          V=sum(m_flow_nominal)*30/1000,
          nPorts=1+nZones)
          annotation (Placement(transformation(extent={{104,-92},{124,-72}})));
        replaceable IDEAS.Fluid.HeatExchangers.ConstantEffectiveness hex(redeclare
            package Medium1 =
              Medium, redeclare package Medium2 = Medium)
          annotation (Placement(transformation(extent={{-146,16},{-126,36}})));
      equation
          // connections that are function of the number of circuits
        for i in 1:nZones loop
          connect(pipeReturnEmission[i].heatPort, fixedTemperature.port) annotation (
              Line(
              points={{138,-88},{138,-68},{-102,-68},{-102,-12},{-127,-12},{-127,-14}},
              color={191,0,0},
              smooth=Smooth.None));
          connect(senTemEm_in.port_b, pumpRad[i].port_a) annotation (Line(
              points={{82,52},{88,52}},
              color={0,127,255},
              smooth=Smooth.None));
        end for;
        // general connections for any configuration
        connect(heatingControl.y, pumpRad.m_flowSet) annotation (Line(
            points={{-119,-70},{100,-70},{100,39.52}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(pipeSupply.heatPort, fixedTemperature.port) annotation (Line(
            points={{-6,54},{-6,42},{-102,42},{-102,6},{-127,6},{-127,-14}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(pipeSupply.port_b, idealCtrlMixer.port_a1) annotation (Line(
            points={{4,58},{34,58}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(emission.port_b, pipeReturnEmission.port_a) annotation (Line(
            points={{150,34},{156,34},{156,-92},{148,-92}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(pumpRad.port_b, emission.port_a) annotation (Line(
            points={{112,52},{116,52},{116,34},{120,34}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(TSet_max.y, ctrl_Heating.TRoo_in1) annotation (Line(
            points={{-169,59.2},{-169,64},{-160.889,64}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(TSet, add.u2) annotation (Line(
            points={{-190,-108},{-190,-75.2},{-175.4,-75.2}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(add.y, heatingControl.u) annotation (Line(
            points={{-159.3,-71},{-146,-71},{-146,-70},{-142,-70}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(idealCtrlMixer.port_b, senTemEm_in.port_a) annotation (Line(
            points={{56,58},{60,58},{60,52},{62,52}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(senTemHea_out.port_b, pipeSupply.port_a) annotation (Line(
            points={{-42,58},{-16,58}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(TSensor, add.u1) annotation (Line(
            points={{-204,-60},{-190,-60},{-190,-66.8},{-175.4,-66.8}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(THigh_val.y, heatingControl.uHigh) annotation (Line(
            points={{-161.4,-52},{-152,-52},{-152,-63.2},{-142,-63.2}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(TLow_val.y, heatingControl.uLow) annotation (Line(
            points={{-159.3,-92},{-152,-92},{-152,-77},{-142,-77}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(senTemEm_out.port_b, spl.port_1) annotation (Line(
            points={{82,-92},{76,-92}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(spl.port_2, pipeReturn.port_a) annotation (Line(
            points={{68,-92},{2,-92}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(idealCtrlMixer.port_a2, spl.port_3) annotation (Line(
            points={{45,46},{44,46},{44,34},{92,34},{92,-78},{72,-78},{72,-88}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(pipeReturnEmission.port_b, vol.ports[1:nZones]) annotation (Line(
            points={{128,-92},{114,-92}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(vol.ports[end], senTemEm_out.port_a) annotation (Line(
            points={{114,-92},{98,-92}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(ctrl_Heating.THeaCur, idealCtrlMixer.TMixedSet) annotation (Line(
            points={{-139.556,69},{-80,69},{-80,80},{45,80},{45,70}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(hex.port_b1, senTemHea_out.port_a) annotation (Line(
            points={{-126,32},{-110,32},{-110,58},{-62,58}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(hex.port_a2, pipeReturn.port_b) annotation (Line(
            points={{-126,20},{-110,20},{-110,-92},{-18,-92}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(pipeReturn.heatPort, fixedTemperature.port) annotation (Line(
            points={{-8,-88},{-10,-88},{-10,-68},{-102,-68},{-102,-12},{-127,-12},{-127,
                -14}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(port_a, hex.port_b2) annotation (Line(
            points={{-100,100},{-100,82},{-180,82},{-180,20},{-146,20}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(heatPortCon, emission.heatPortCon) annotation (Line(
            points={{-200,20},{-182,20},{-182,86},{142.5,86},{142.5,44}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(heatPortRad, emission.heatPortRad) annotation (Line(
            points={{-200,-20},{-186,-20},{-186,90},{148.5,90},{148.5,44}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(absolutePressure.ports[1], pipeReturn.port_b) annotation (Line(
            points={{-106,-104},{-106,-92},{-18,-92}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(hex.port_a1, port_b) annotation (Line(
            points={{-146,32},{-178,32},{-178,94},{-120,94},{-120,100},{-120,100}},
            color={0,127,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                  -100},{200,100}}), graphics={Rectangle(
                extent={{-98,30},{88,-64}},
                lineColor={135,135,135},
                lineThickness=1), Text(
                extent={{36,30},{86,20}},
                lineColor={135,135,135},
                lineThickness=1,
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid,
                textString="Thermal Energy Storage")}));
      end PartialDistrictHeating;
    end Interfaces;
  end HeatingSystem;
  annotation (uses(IDEAS(version="0.1"), Modelica(version="3.2.1")));
  package Interfaces
    partial model HeatingSystem "Partial heating/cooling system"

      outer IDEAS.SimInfoManager sim
        "Simulation information manager for climate data"
        annotation (Placement(transformation(extent={{-200,80},{-180,100}})));

      // *********** Building characteristics and  interface ***********
      // --- General
      parameter Integer nZones(min=1)
        "Number of conditioned thermal zones in the building";
      parameter Boolean isHea=true "true if system is able to heat";
      parameter Boolean isCoo=false "true if system is able to cool";
      parameter Boolean use_DHW = sim.DHW;
      parameter Boolean DH=false "true if system is connected to a DH grid";

      // --- Ports
      parameter Integer nConvPorts(min=0)
        "Number of ports in building for convective heating/cooling";
      parameter Integer nRadPorts(min=0)
        "Number of ports in building for radiative heating/cooling";
      parameter Integer nEmbPorts(min=0)
        "Number of ports in building for embedded systems";

      // --- Electrical
      parameter Integer nLoads(min=0) = 1
        "Number of electric loads. If zero, all electric equations disappear.";

      // --- Sensor
      parameter Integer nTemSen(min=0)
        "number of temperature inputs for the system";

      // *********** Outputs ***********
      // --- Thermal
      Modelica.SIunits.Power QHeaSys if isHea
        "Total energy use forspace heating + DHW, if present)";
      Modelica.SIunits.Power QCooTotal if isCoo "Total cooling energy use";

      // *********** Interface ***********
      // --- thermal
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[nConvPorts] heatPortCon
        "Nodes for convective heat gains"
        annotation (Placement(transformation(extent={{-210,10},{-190,30}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[nRadPorts] heatPortRad
        "Nodes for radiative heat gains"
        annotation (Placement(transformation(extent={{-210,-30},{-190,-10}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b[nEmbPorts] heatPortEmb
        "Construction nodes for heat gains by embedded layers"
        annotation (Placement(transformation(extent={{-210,50},{-190,70}})));

      // --- Hydraulic
      Modelica.Fluid.Interfaces.FluidPort_a port_a if DH
        annotation (Placement(transformation(extent={{-110,90},{-90,110}})));
      Modelica.Fluid.Interfaces.FluidPort_b port_b if DH
        annotation (Placement(transformation(extent={{-130,90},{-110,110}})));

      // --- Electrical
      Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug
        plugLoad(m=1) if nLoads >= 1
        "Electricity connection to the Inhome feeder"
        annotation (Placement(transformation(extent={{190,-10},{210,10}})));
      IDEAS.Electric.BaseClasses.WattsLawPlug wattsLawPlug(each numPha=1,
          final nLoads=nLoads) if nLoads >= 1
        annotation (Placement(transformation(extent={{168,-10},{188,10}})));

      // --- Sensor
      Modelica.Blocks.Interfaces.RealInput[nTemSen] TSensor(final quantity="ThermodynamicTemperature",unit="K",displayUnit="degC", min=0)
        "Sensor temperature"
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=180,
            origin={-204,-60})));

      //   parameter Modelica.SIunits.Power[nZones] QNom(each min=0) = ones(nZones)*5000
      //     "Nominal power, can be seen as the max power of the emission system";
      //   parameter Real[nZones] VZones "Conditioned volumes of the zones";
      //   final parameter Modelica.SIunits.HeatCapacity[nZones] C=1012*1.204*VZones*5
      //     "Heat capacity of the conditioned zones";
      //
         Modelica.Blocks.Interfaces.RealInput[nZones] TSet
        "Setpoint temperature for the zones"    annotation (Placement(transformation(
               extent={{-10,-10},{10,10}},
               rotation=90,
               origin={0,-104})));

    protected
      final parameter Integer nLoads_min = max(1,nLoads);
       Modelica.SIunits.Power[nLoads_min] P
        "dummy variable active power for each of the loads";
       Modelica.SIunits.Power[nLoads_min] Q
        "dummy variable passive power for each of the loads";
    public
      Modelica.Blocks.Sources.RealExpression[nLoads_min] realExpression(y=P)
        annotation (Placement(transformation(extent={{146,0},{160,16}})));
      Modelica.Blocks.Sources.RealExpression[nLoads_min] realExpression1(y=Q)
        annotation (Placement(transformation(extent={{148,-14},{162,4}})));
         Modelica.Blocks.Interfaces.RealInput mDHW60C if use_DHW
        "Setpoint temperature for the zones" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={60,-104})));

    equation
      if nLoads >= 1 then
         connect(wattsLawPlug.vi, plugLoad) annotation (Line(
          points={{188,0},{200,0}},
          color={85,170,255},
          smooth=Smooth.None));
         connect(realExpression.y, wattsLawPlug.P) annotation (Line(
          points={{160.7,8},{163.35,8},{163.35,6},{168,6}},
          color={0,0,127},
          smooth=Smooth.None));
        connect(realExpression1.y, wattsLawPlug.Q) annotation (Line(
              points={{162.7,-5},{164,-5},{164,2},{168,2}},
              color={0,0,127},
              smooth=Smooth.None));
      end if;

      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{200,100}}),
            graphics={
            Rectangle(
              extent={{-200,100},{200,-100}},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid,
              lineColor={191,0,0}),
            Polygon(
              points={{-46,-8},{-46,-20},{-44,-22},{-24,-10},{-24,2},{-26,4},{-46,-8}},
              lineColor={127,0,0},
              smooth=Smooth.None,
              fillColor={127,0,0},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-46,-32},{-46,-44},{-44,-46},{-24,-34},{-24,-22},{-26,-20},{-46,
                  -32}},
              lineColor={127,0,0},
              smooth=Smooth.None,
              fillColor={127,0,0},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-44,-18},{-50,-22},{-50,-46},{-46,-50},{28,-50},{42,-40}},
              color={127,0,0},
              smooth=Smooth.None),
            Line(
              points={{-50,-46},{-44,-42}},
              color={127,0,0},
              smooth=Smooth.None),
            Line(
              points={{-24,0},{-20,2},{-20,-32},{-16,-36},{-16,-36},{40,-36}},
              color={127,0,0},
              smooth=Smooth.None),
            Line(
              points={{-24,-24},{-20,-22}},
              color={127,0,0},
              smooth=Smooth.None),
            Polygon(
              points={{40,-26},{40,-46},{50,-52},{58,-46},{58,-30},{54,-24},{48,-20},
                  {40,-26}},
              lineColor={127,0,0},
              smooth=Smooth.None,
              fillColor={127,0,0},
              fillPattern=FillPattern.Solid),
            Line(
              points={{200,100},{200,-100}},
              color={85,170,255},
              smooth=Smooth.None)}),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{200,
                100}}), graphics),
        Documentation(info="<html>
<p><b>Description</b> </p>
<p>Interface model for a complete multi-zone heating system (with our without domestic hot water and solar system).</p>
<p>This model defines the ports used to link a heating system with a building, and the basic parameters that most heating systems will need to have. The model is modular as a function of the number of zones <i>nZones. </i></p>
<p>Two sets of heatPorts are defined:</p>
<p><ol>
<li><i>heatPortCon[nZones]</i> and <i>heatPortRad[nZones]</i> for convective respectively radiative heat transfer to the building. </li>
<li><i>heatPortEmb[nZones]</i> for heat transfer to TABS elements in the building. </li>
</ol></p>
<p>The model also defines <i>TSensor[nZones]</i> and <i>TSet[nZones]</i> for the control, and a nominal power <i>QNom[nZones].</i></p>
<p>There is also an input for the DHW flow rate, <i>mDHW60C</i>, but this can be unconnected if the system only includes heating and no DHW.</p>
<p><h4>Assumptions and limitations </h4></p>
<p><ol>
<li>See the different extensions of this model in <a href=\"modelica://IDEAS.Thermal.HeatingSystems\">IDEAS.Thermal.HeatingSystems</a></li>
</ol></p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Connect the heating system to the corresponding heatPorts of a <a href=\"modelica://IDEAS.Interfaces.BaseClasses.Structure\">structure</a>. </li>
<li>Connect <i>TSet</i> and <i>TSensor</i> and <i>plugLoad. </i></li>
<li>Connect <i>plugLoad </i> to an inhome grid.  A<a href=\"modelica://IDEAS.Interfaces.BaseClasses.CausalInhomeFeeder\"> dummy inhome grid like this</a> has to be used if no inhome grid is to be modelled. </li>
<li>Set all parameters that are required, depending on which implementation of this interface is used. </li>
</ol></p>
<p><h4>Validation </h4></p>
<p>No validation performed.</p>
<p><h4>Example </h4></p>
<p>See the <a href=\"modelica://IDEAS.Thermal.HeatingSystems.Examples\">heating system examples</a>. </p>
</html>"));
    end HeatingSystem;
  end Interfaces;
end DistrictHeating;
