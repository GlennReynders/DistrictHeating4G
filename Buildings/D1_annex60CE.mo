within DistrictHeating4G.Buildings;
model D1_annex60CE "A60 Detached V1"

  extends IDEAS.Interfaces.BaseClasses.Structure(
    nZones=3,
    final AZones={92.7,92.7,49.3},
    final VZones={236.4,236.4,55.7},
    final nEmb=nZones,
    final ATrans);

  parameter Modelica.SIunits.Angle orientation = 0
    "Tilt compared to original orientation (=0)";
  parameter Real f=0.15 " Frame/window fraction: 0.21 ";

   //Attic
protected
  IDEAS.Buildings.Components.Zone Attic(
    corrCV=5,
    nSurf=5,
    V=55.74,
    n50=2) "Attic"
    annotation (Placement(transformation(extent={{80,50},{100,70}})));
  IDEAS.Buildings.Components.OuterWall roof_A[2](
    azi={IDEAS.Constants.South + orientation,IDEAS.Constants.North +
        orientation},
    redeclare CE.Data.Constructions.State1.S1Roof constructionType,
    AWall={32.5,50.7},
    each insulationThickness=0,
    inc={-0.5235987755983,0.5235987755983}) "Roof of attic"
                                      annotation (Placement(
        transformation(
        extent={{-5,-10},{5,10}},
        rotation=90,
        origin={-29,46})));
  IDEAS.Buildings.Components.OuterWall walls_A[2](
    azi={IDEAS.Constants.West + orientation,IDEAS.Constants.East + orientation},
    redeclare CE.Data.Constructions.State1.S1Wall constructionType,
    AWall={6.29,6.29},
    each insulationThickness=0,
    inc={1.5707963267949,1.5707963267949}) "Outer walls of attic"
    annotation (Placement(transformation(
        extent={{-5,-10},{5,10}},
        rotation=90,
        origin={11,46})));

  //Nightzone

  IDEAS.Buildings.Components.Zone nightzone(
    corrCV=5,
    nSurf=14,
    V=236.4,
    n50=2) "Nightzone (bedrooms, corridors & bathroom)"
    annotation (Placement(transformation(extent={{80,-10},{100,10}})));

  IDEAS.Buildings.Components.OuterWall roof_N[2](
    azi={IDEAS.Constants.South + orientation,IDEAS.Constants.North +
        orientation},
    redeclare CE.Data.Constructions.State1.S1Roof constructionType,
    AWall={13.2,13.2},
    each insulationThickness=0,
    inc={-0.5235987755983,0.5235987755983}) "Roof of nightzone"
                                                annotation (Placement(
        transformation(
        extent={{-5,-10},{5,10}},
        rotation=90,
        origin={-29,-14})));

  IDEAS.Buildings.Components.InternalWall int_wall_N(
    azi=0,
    redeclare CE.Data.Constructions.State1.S1Wall_Int constructionType,
    AWall=56.5,
    insulationThickness=0,
    inc=1.5707963267949) "Walls in the nightzone" annotation (Placement(
        transformation(
        extent={{-5,-9},{5,9}},
        rotation=90,
        origin={-69,-13})));

  IDEAS.Buildings.Components.OuterWall walls_N[4](
    azi={IDEAS.Constants.South + orientation,IDEAS.Constants.West + orientation,
        IDEAS.Constants.North + orientation,IDEAS.Constants.East + orientation},
    redeclare CE.Data.Constructions.State1.S1Wall constructionType,
    AWall={31.8,18.2,36.1,18},
    each insulationThickness=0,
    inc={1.5707963267949,1.5707963267949,1.5707963267949,1.5707963267949})
    "Outer walls of nightzone"
    annotation (Placement(transformation(
        extent={{-5,-10},{5,10}},
        rotation=90,
        origin={11,-14})));

  IDEAS.Buildings.Components.Window windows_N[4](
    redeclare IDEAS.Buildings.Components.Shading.None shaType,
    azi={IDEAS.Constants.South + orientation,IDEAS.Constants.West + orientation,
        IDEAS.Constants.North + orientation,IDEAS.Constants.East + orientation},
    redeclare CE.Data.Glazing.Ins2 glazing,
    redeclare CE.Data.Frames.FrameU2_5 fraType,
    frac={f,f,f,f},
    A={4.3,2.7,0.8,2.9},
    inc={1.5707963267949,1.5707963267949,1.5707963267949,1.5707963267949})
    "Windows of nightzone"
    annotation (Placement(transformation(
        extent={{-5,-10},{5,10}},
        rotation=90,
        origin={51,-14})));

  //Dayzone

  IDEAS.Buildings.Components.Zone dayzone(
    corrCV=5,
    nSurf=14,
    V=236.4,
    n50=2) "Dayzone (living room & kitchen)"
    annotation (Placement(transformation(extent={{80,-70},{100,-50}})));
  IDEAS.Buildings.Components.SlabOnGround floor_D(
    inc=0,
    azi=0,
    redeclare CE.Data.Constructions.State1.S1Floor constructionType,
    AWall=71.3,
    PWall=40.5,
    insulationThickness=0) "Slab on ground of dayzone"
                                                annotation (Placement(
        transformation(
        extent={{-5,-10},{5,10}},
        rotation=90,
        origin={-29,-74})));
  IDEAS.Buildings.Components.InternalWall int_wall_D(
    azi=0,
    redeclare CE.Data.Constructions.State1.S1Wall_Int constructionType,
    AWall=58.65,
    insulationThickness=0,
    inc=1.5707963267949) "Walls in the dayzone" annotation (Placement(
        transformation(
        extent={{-5.5,-9.5},{5.5,9.5}},
        rotation=90,
        origin={-69.5,-72.5})));
  IDEAS.Buildings.Components.OuterWall walls_D[4](
    azi={IDEAS.Constants.South + orientation,IDEAS.Constants.West +
        orientation,IDEAS.Constants.North + orientation,IDEAS.Constants.East
         + orientation},
    redeclare CE.Data.Constructions.State1.S1Wall constructionType,
    AWall={24.76,15.26,30,16.1},
    each insulationThickness=0,
    inc={1.5707963267949,1.5707963267949,1.5707963267949,1.5707963267949})
    "Outer walls of dayzone" annotation (Placement(transformation(
        extent={{-5,-10},{5,10}},
        rotation=90,
        origin={11,-74})));
  IDEAS.Buildings.Components.OuterWall doors_D[2](
    redeclare CE.Data.Constructions.State1.S1Door constructionType,
    azi={IDEAS.Constants.North + orientation,IDEAS.Constants.West + orientation},
    redeclare IDEAS.Buildings.Data.Insulation.none insulationType,
    AWall={1.8,1.8},
    each insulationThickness=0,
    inc={1.5707963267949,1.5707963267949}) "Exterior doors of dayzone"
                                                     annotation (
      Placement(transformation(
        extent={{-5,-10},{5,10}},
        rotation=90,
        origin={51,-94})));
  IDEAS.Buildings.Components.Window windows_D[4](
    redeclare IDEAS.Buildings.Components.Shading.None shaType,
    azi={IDEAS.Constants.South + orientation,IDEAS.Constants.West + orientation,
        IDEAS.Constants.North + orientation,IDEAS.Constants.East + orientation},
    redeclare CE.Data.Glazing.Ins2 glazing,
    redeclare CE.Data.Frames.FrameU2_5 fraType,
    frac={f,f,f,f},
    A={7.04,5.64,1.8,4.8},
    inc={1.5707963267949,1.5707963267949,1.5707963267949,1.5707963267949})
    "Windows of dayzone" annotation (Placement(transformation(
        extent={{-5,-10},{5,10}},
        rotation=90,
        origin={51,-74})));

  //Floors and walls between zones

  IDEAS.Buildings.Components.InternalWall int_floor_N_A(
    inc=0,
    azi=0,
    redeclare CE.Data.Constructions.State1.S1Floor_Int constructionType,
    AWall=71.3,
    insulationThickness=0) "Internal floors between day and night zones"
    annotation (Placement(transformation(
        extent={{-5,-10},{5,10}},
        rotation=90,
        origin={71,20})));

  IDEAS.Buildings.Components.InternalWall int_floor_N_A1(
    inc=0,
    azi=0,
    redeclare CE.Data.Constructions.State1.S1Floor_Int constructionType,
    AWall=71.3,
    insulationThickness=0) "Internal floors between day and night zones"
    annotation (Placement(transformation(
        extent={{-5,-10},{5,10}},
        rotation=90,
        origin={73,-36})));
equation
  connect(dayzone.TSensor, TSensor[1]) annotation (Line(
      points={{100.6,-60},{134,-60},{134,-66.6667},{156,-66.6667}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(nightzone.TSensor, TSensor[2]) annotation (Line(
      points={{100.6,0},{134,0},{134,-60},{156,-60}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(Attic.TSensor, TSensor[3]) annotation (Line(
      points={{100.6,60},{134,60},{134,-54},{140,-54},{140,-53.3333},{156,
          -53.3333}},
      color={0,0,127},
      smooth=Smooth.None));

  connect(Attic.gainCon, heatPortCon[3]) annotation (Line(
      points={{100,57},{116,57},{116,28},{122,28},{122,26.6667},{150,26.6667}},
      color={191,0,0},
      smooth=Smooth.None));

  connect(Attic.gainRad, heatPortRad[3]) annotation (Line(
      points={{100,54},{124,54},{124,-12},{136,-12},{136,-13.3333},{150,
          -13.3333}},
      color={191,0,0},
      smooth=Smooth.None));

  connect(nightzone.gainCon, heatPortCon[2]) annotation (Line(
      points={{100,-3},{108,-3},{116,-3},{116,-4},{116,20},{150,20}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(nightzone.gainRad, heatPortRad[2]) annotation (Line(
      points={{100,-6},{124,-6},{124,-20},{150,-20}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(dayzone.gainCon, heatPortCon[1]) annotation (Line(
      points={{100,-63},{108,-63},{116,-63},{116,-64},{116,14},{130,14},{130,
          13.3333},{150,13.3333}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(dayzone.gainRad, heatPortRad[1]) annotation (Line(
      points={{100,-66},{124,-66},{124,-26},{142,-26},{142,-26.6667},{150,
          -26.6667}},
      color={191,0,0},
      smooth=Smooth.None));

  connect(int_floor_N_A.port_emb, heatPortEmb[3]) annotation (Line(
      points={{81,20},{110,20},{110,40},{140,40},{140,60},{150,60}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(floor_D.port_emb, heatPortEmb[1]) annotation (Line(
      points={{-19,-74},{-6,-74},{-6,-28},{110,-28},{110,40},{140,40},{140,60},{
          150,60}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(roof_A.propsBus_a, Attic.propsBus[1:2]) annotation (Line(
      points={{-33,51},{-33,64.8},{80,64.8}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(walls_A.propsBus_a, Attic.propsBus[3:4]) annotation (Line(
      points={{7,51},{7,63.2},{80,63.2}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(int_floor_N_A.propsBus_a, Attic.propsBus[5]) annotation (Line(
      points={{67,25},{67,54},{80,54},{80,62.4}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(int_floor_N_A.propsBus_b, nightzone.propsBus[1]) annotation (Line(
      points={{67,15},{67,5.85714},{80,5.85714}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(int_wall_N.propsBus_b, nightzone.propsBus[2]) annotation (Line(
      points={{-72.6,-18},{-74,-18},{-74,-24},{-84,-24},{-84,5.57143},{80,
          5.57143}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));

  connect(int_wall_N.propsBus_a, nightzone.propsBus[3]) annotation (Line(
      points={{-72.6,-8},{-74,-8},{-74,5.28571},{80,5.28571}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(roof_N.propsBus_a, nightzone.propsBus[4:5]) annotation (Line(
      points={{-33,-9},{-33,4.71429},{80,4.71429}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(walls_N.propsBus_a, nightzone.propsBus[6:9]) annotation (Line(
      points={{7,-9},{7,3.57143},{80,3.57143}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(windows_N.propsBus_a, nightzone.propsBus[10:13]) annotation (Line(
      points={{47,-9},{48,-9},{48,2},{80,2},{80,2.42857}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(int_wall_D.propsBus_b, dayzone.propsBus[1]) annotation (Line(
      points={{-73.3,-78},{-74,-78},{-74,-84},{-82,-84},{-82,-54.1429},{80,
          -54.1429}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));

  connect(int_wall_D.propsBus_a, dayzone.propsBus[2]) annotation (Line(
      points={{-73.3,-67},{-73.3,-54.4286},{80,-54.4286}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(floor_D.propsBus_a, dayzone.propsBus[3]) annotation (Line(
      points={{-33,-69},{-33,-54.7143},{80,-54.7143}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(walls_D.propsBus_a, dayzone.propsBus[4:7]) annotation (Line(
      points={{7,-69},{7,-55.8571},{80,-55.8571}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(windows_D.propsBus_a, dayzone.propsBus[8:11]) annotation (Line(
      points={{47,-69},{47,-57},{80,-57}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(doors_D.propsBus_a, dayzone.propsBus[12:13]) annotation (Line(
      points={{47,-89},{47,-82},{70,-82},{70,-70},{70,-70},{70,-56},{70,-56},{
          70,-56},{80,-56},{80,-57.5714}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(int_floor_N_A1.port_emb, heatPortEmb[2]) annotation (Line(
      points={{83,-36},{116,-36},{116,60},{150,60}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(int_floor_N_A1.propsBus_a, nightzone.propsBus[14]) annotation (Line(
      points={{69,-31},{69,-13.5},{80,-13.5},{80,2.14286}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(int_floor_N_A1.propsBus_b, dayzone.propsBus[14]) annotation (Line(
      points={{69,-41},{69,-48.5},{80,-48.5},{80,-57.8571}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(flowPort_In[3], Attic.flowPort_In) annotation (Line(
      points={{20,106.667},{56,106.667},{56,70},{92,70}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(flowPort_In[2], nightzone.flowPort_In) annotation (Line(
      points={{20,100},{56,100},{56,10},{92,10}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(flowPort_In[1], dayzone.flowPort_In) annotation (Line(
      points={{20,93.3333},{56,93.3333},{56,-50},{92,-50}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(flowPort_Out[3], Attic.flowPort_Out) annotation (Line(
      points={{-20,106.667},{34,106.667},{34,70},{88,70}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(flowPort_Out[2], nightzone.flowPort_Out) annotation (Line(
      points={{-20,100},{33,100},{33,10},{88,10}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(flowPort_Out[1], dayzone.flowPort_Out) annotation (Line(
      points={{-20,93.3333},{34,93.3333},{34,-50},{88,-50}},
      color={0,0,0},
      smooth=Smooth.None));
  annotation (
    Line(
      points={{27,131},{27,140},{70,140},{70,134},{80,134}},
      color={0,0,255},
      smooth=Smooth.None),
    Line(
      points={{65,44},{68,44},{68,50},{92,50},{92,120}},
      color={191,0,0},
      smooth=Smooth.None),
    Diagram(coordinateSystem(extent={{-120,-100},{160,80}},
          preserveAspectRatio=false), graphics),
    Icon(coordinateSystem(extent={{-120,-100},{160,80}})));
end D1_annex60CE;
