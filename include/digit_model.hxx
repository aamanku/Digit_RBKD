#ifndef DIGIT_MODEL_HXX
#define DIGIT_MODEL_HXX

#include "rigid_body.hxx"
// #include "tinyxml2.h"
#include <chrono>

using namespace rbda;

/*
<worldbody>
    <light mode='trackcom' name='sun' pos='0 0 5' dir='1 0 -3' specular='0.7 0.7 0.3' exponent='0.5' directional='true' castshadow='false'/>
    <geom name='floor' type='plane' size='0 0 1' conaffinity='127' condim='3' friction='0.7 0.01 0.005' pos='0 0 0' priority='100' />
    <body name='base' pos='0 0 0' euler='-0 0 -0'>
      <inertial pos='0.001636612541 0.0002001180789 0.2593064529' mass='15.028392' fullinertia='0.3759052548 0.3441935 0.09873232746 -8.776732849e-05 0.008498611229 6.621611757e-05' />
      <joint type='free' limited='false'/>
      <site name='imu' pos='0 0 0' euler='0 -90 0'/>
      <site name='context' pos='0.06425 0 0.38' euler='180 0 0' size='0.01'/>
      <geom type='mesh' mesh='torso-base-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
      <geom type='mesh' mesh='torso-base-1' pos='-0.001 0.091 0' euler='0 -90 0' material='robot'/>
      <geom type='mesh' mesh='torso-base-2' pos='-0.001 0.12 0.4' euler='-9.999933098 -90 0' material='robot'/>
      <geom type='mesh' mesh='torso-base-3' pos='-0.001 -0.091 0' euler='0 -90 0' material='robot'/>
      <geom type='mesh' mesh='torso-base-4' pos='-0.001 -0.12 0.4' euler='9.999933098 -90 0' material='robot'/>
      <camera name='forward-tis-dfm27up' fovy='49.6875' pos='0.06425 0 0.38' euler='90 -90 0' user='1280 960 1'/>
      <camera name='forward-chest-realsense' fovy='60.2875' pos='0.093981 0.0225 0.426449' euler='-0 -45.00006627 -90' user='848 480 0'/>
      <camera name='downward-pelvis-realsense-d430' fovy='60.2875' pos='0.0305 0.025 -0.03268' euler='-0 0 -90' user='848 480 0'/>
      <camera name='forward-pelvis-realsense-d430' fovy='60.2875' pos='0.061607 0.025 -0.025283' euler='-0 -45.00006627 -90' user='848 480 0'/>
      <camera name='backward-pelvis-realsense-d430' fovy='60.2875' pos='-0.000607 0.025 -0.025283' euler='7.016725531e-15 45.00006627 -90' user='848 480 0'/>

      <geom type='box' size='0.0975 0.087 0.245' pos='0.008 0 0.21' euler='-0 0 -0' friction='0.7 0.01 0.005'  mass='15.028392' rgba='0.5 0.5 0.5 0.0' class='collision'/>
      <geom type='cylinder' size='0.05 0.0325' pos='0.02 0 0.49' euler='-0 0 -0' friction='0.7 0.01 0.005'  mass='15.028392' rgba='0.5 0.5 0.5 0.0' class='collision'/>
      <body name='left-hip-roll' pos='-0.001 0.091 0' euler='21.49994484 -90 0'>
        <inertial pos='-0.001967 0.000244 0.031435' mass='0.915088' fullinertia='0.001017 0.001148 0.000766 -3e-06 1.3e-05 -4e-06' />
        <joint name='left-hip-roll' type='hinge' axis='0 0 1' range='-60 60' stiffness='0' frictionloss='1' damping='1' armature='0.173823936' limited='true' pos='0 0 0'/>
        <geom type='mesh' mesh='left-leg-hip-roll-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
        <body name='left-hip-yaw' pos='-0.0505 0 0.044' euler='0 -90 0'>
          <inertial pos='1e-05 -0.001945 0.042033' mass='0.818753' fullinertia='0.001627 0.001929 0.00077 -1e-06 2e-06 5.3e-05' />
          <joint name='left-hip-yaw' type='hinge' axis='0 0 1' range='-40 40' stiffness='0' frictionloss='1' damping='1' armature='0.067899975' limited='true' pos='0 0 0'/>
          <geom type='mesh' mesh='left-leg-hip-yaw-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
          <body name='left-hip-pitch' pos='0 0.004 0.068' euler='90 -2.862499213e-14 134.9999776'>
            <inertial pos='0.060537 0.000521 -0.038857' mass='6.244279' fullinertia='0.011533 0.033345 0.033958 -0.000171 0.000148 0.000178' />
            <joint name='left-hip-pitch' type='hinge' axis='0 0 -1' range='-60 90' stiffness='0' frictionloss='0.5' damping='1' armature='0.1204731904' limited='true' pos='0 0 0'/>
            <geom type='mesh' mesh='left-leg-hip-pitch-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
            <geom type='cylinder' size='0.05 0.072' pos='0 0 -0.04' euler='-0 0 -0' friction='0.7 0.01 0.005' contype='1' conaffinity='-2' mass='6.244279' rgba='0.5 0.5 0.5 0.0' class='collision'/>
            <geom type='cylinder' size='0.05 0.072' pos='0.12 0 -0.04' euler='-0 0 -0' friction='0.7 0.01 0.005' contype='1' conaffinity='-2' mass='6.244279' rgba='0.5 0.5 0.5 0.0' class='collision'/>
            <geom type='box' size='0.06 0.05 0.072' pos='0.06 0 -0.04' euler='-0 0 -0' friction='0.7 0.01 0.005' contype='1' conaffinity='-2' mass='6.244279' rgba='0.5 0.5 0.5 0.0' class='collision'/>
            <site name='lhp_arod' pos='0 0 0.046' euler='4.09 0.05 -82.27' size='0.01'/>
            <body name='left-knee' pos='0.12 0 0.0045' euler='-0 0 -90'>
              <inertial pos='0.045641 0.042154 0.001657' mass='1.227077' fullinertia='0.002643 0.005098 0.007019 -0.001832 6.6e-05 4.5e-05' />
              <joint name='left-knee' type='hinge' axis='0 0 1' range='-80 58.4' stiffness='0' frictionloss='0.5' damping='1' armature='0.1204731904' limited='true' pos='0 0 0'/>
              <geom type='mesh' mesh='left-leg-knee-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
              <body name='left-shin' pos='0.060677 0.047406 0' euler='-0 0 -0'>
                <!-- <inertial pos='0.174265 0.010265 0.00107' mass='0.895793' fullinertia='0.001128 0.022492 0.022793 0.001098 0.000196 -3e-06' /> -->
                <inertial pos='0.17637939 -0.01361614  0.00483379' mass='1.073941' fullinertia='0.00439118 0.02701227 0.03037536 0.0007727 0.00046775 0.00052522' />
                <joint name='left-shin' type='hinge' axis='0 0 1'  stiffness='6000' frictionloss='0' damping='0' armature='0' limited='false' pos='0 0 0'/>
                <geom type='mesh' mesh='left-leg-shin-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                <geom type='cylinder' size='0.03 0.1715' pos='0.23 0 0' euler='0 90 0' friction='0.7 0.01 0.005' contype='1' conaffinity='-2' mass='0.895793' rgba='0.5 0.5 0.5 0.0' class='collision'/>
                <geom type='cylinder' size='0.05 0.1' pos='0.125 0.009 0' euler='90 83.99999778 9.128289303e-14' friction='0.7 0.01 0.005' contype='1' conaffinity='-2' mass='0.895793' rgba='0.5 0.5 0.5 0.0' class='collision'/>
                <body name='left-tarsus' pos='0.434759 0.02 0' euler='-0 0 102.9999562'>
                  <!-- <inertial pos='0.100777 -0.029183 0.000678' mass='1.322865' fullinertia='0.000932 0.016409 0.016501 0.00061 0.000102 9e-06' /> -->
                  <inertial pos='0.11692845 -0.03641395  0.00050495' mass='1.493355' fullinertia='1.60070790e-03 2.14664590e-02 2.21038310e-02 1.98205783e-03 8.97031761e-05 -5.41167493e-06' />
                  <joint name='left-tarsus' type='hinge' axis='0 0 1' range='-50.3 71.6' stiffness='0' frictionloss='0' damping='0' armature='0' limited='true' pos='0 0 0'/>
                  <geom type='mesh' mesh='left-leg-tarsus-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                  <geom type='capsule' size='0.02 0.18' pos='0.21 -0.03 0' euler='0 90 0' friction='0.7 0.01 0.005' contype='1' conaffinity='-2' mass='1.322865' rgba='0.5 0.5 0.5 0.0' class='collision'/>
                  <geom type='box' size='0.007 0.07 0.03' pos='-0.07 -0.012 0' euler='4.999998056 -4.600265621e-05 64.9999639' friction='0.7 0.01 0.005' contype='1' conaffinity='-2' mass='1.322865' rgba='0.5 0.5 0.5 0.0' class='collision'/>
                  <body name='left-heel-spring' pos='-0.01766 -0.029456 0.00104' euler='4.470028555 0.3199786449 155.799968'>
                    <inertial pos='0.049086 0.004739 -4.5e-05' mass='0.230018' fullinertia='5.5e-05 0.00074 0.000701 1.5e-05 1e-06 0' />
                    <joint name='left-heel-spring' type='hinge' axis='0 0 1' range='-6 6' stiffness='4375' frictionloss='0' damping='0' armature='0' limited='true' pos='0 0 0'/>
                    <geom type='mesh' mesh='left-leg-heel-spring-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                    <site name='lhs-arod' pos = '0.113789 -0.011056 0' euler='0 0 0' size='0.01'/>
                  </body>
                  <body name='left-toe-A' pos='0.059 -0.034 -0.0276' euler='180 -7.015553385e-15 88.9599851'>
                    <inertial pos='0.005161 1e-06 -0.002248' mass='0.139557' fullinertia='2.9e-05 5.8e-05 7.4e-05 0 -4e-06 0' />
                    <joint name='left-toe-A' type='hinge' axis='0 0 1' range='-46.2755 44.9815' stiffness='0' frictionloss='1' damping='1' armature='0.036089474999999996' limited='true' pos='0 0 0'/>
                    <geom type='mesh' mesh='left-leg-toe-a-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                    <site name='ltA-rod' pos='0.057 0 -0.008' euler='0.34 2.14 -91.47' size='0.01'/>
                  </body>
                  <body name='left-toe-B' pos='0.111 -0.034 0.0276' euler='-0 0 -88.9599851'>
                    <inertial pos='0.005161 1e-06 -0.002248' mass='0.139557' fullinertia='2.9e-05 5.8e-05 7.4e-05 0 -4e-06 0' />
                    <joint name='left-toe-B' type='hinge' axis='0 0 1' range='-45.8918 45.5476' stiffness='0' frictionloss='1' damping='1' armature='0.036089474999999996' limited='true' pos='0 0 0'/>
                    <geom type='mesh' mesh='left-leg-toe-b-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                    <site name='ltB-rod' pos='0.057 0 -0.008' euler='-0.22 -3.08 91.38' size='0.01'/>
                  </body>
                  <body name='left-toe-pitch' pos='0.408 -0.04 0' euler='-0 0 71.75004157'>
                    <inertial pos='-0.000141 2e-06 3e-06' mass='0.043881' fullinertia='5e-06 8e-06 4e-06 0 0 0' />
                    <joint name='left-toe-pitch' type='hinge' axis='0 0 1' range='-44 34' stiffness='0' frictionloss='0' damping='0' armature='0' limited='true' pos='0 0 0'/>
                    <geom type='mesh' mesh='left-leg-toe-pitch-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                    <body name='left-toe-roll' pos='0 0 0' euler='0 90 0'>
                      <inertial pos='9e-06 -0.028084 -0.023204' mass='0.531283' fullinertia='0.00187 0.001616 0.000843 0 0 0.000566' />
                      <joint name='left-toe-roll' type='hinge' axis='0 0 1' range='-37 33' stiffness='0' frictionloss='0' damping='0' armature='0' limited='true' pos='0 0 0'/>
                      <geom type='mesh' mesh='left-leg-toe-roll-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                      <geom type='box' size='0.04 0.1175 0.0115' pos='0 -0.0437 -0.0255' euler='-60.00002314 0 -0' friction='0.7 0.01 0.005' contype='1' conaffinity='-2' mass='0.531283' rgba='0.5 0.5 0.5 0.0' class='collision'/>
                      <site name='left-foot' pos='0 -0.05456 -0.0315' euler='-60 0 -90' size='0.01'/>
                      <site name='ltrA-rod' pos='0.0179 -0.009551 -0.054164' euler='0 0 0' size='0.01'/>
                      <site name='ltrB-rod' pos='-0.0181 -0.009551 -0.054164' euler='0 0 0' size='0.01'/>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
      <body name='left-shoulder-roll' pos='-0.001 0.12 0.4' euler='-9.999933098 -90 0'>
        <inertial pos='-0.000819 -0.003158 0.023405' mass='0.535396' fullinertia='0.000704 0.00075 0.000298 1.4e-05 1.2e-05 3.5e-05' />
        <joint name='left-shoulder-roll' type='hinge' axis='0 0 1' range='-75 75' stiffness='0' frictionloss='2' damping='2' armature='0.173823936' limited='true' pos='0 0 0'/>
        <geom type='mesh' mesh='left-arm-shoulder-roll-0' pos='0 0 0' euler='-0 0 -15.99998964' material='robot'/>
        <body name='left-shoulder-pitch' pos='-0.00317 -0.011055 0.0555' euler='89.99999034 -16.0000415 -44.99997193'>
          <inertial pos='-4.2e-05 -0.061882 -0.073788' mass='1.440357' fullinertia='0.006761 0.002087 0.005778 -6e-06 -3e-06 -0.002046' />
          <joint name='left-shoulder-pitch' type='hinge' axis='0 0 -1' range='-145 145' stiffness='0' frictionloss='2' damping='2' armature='0.173823936' limited='true' pos='0 0 0'/>
          <geom type='mesh' mesh='left-arm-shoulder-pitch-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
          <geom type='cylinder' size='0.038 0.07' pos='0 -0.093 -0.1' euler='90 0 -0' friction='0.7 0.01 0.005' contype='2' conaffinity='-3' mass='1.440357' rgba='0.5 0.5 0.5 0.0' class='collision'/>
          <body name='left-shoulder-yaw' pos='0 -0.165 -0.1' euler='90 0 -0'>
            <inertial pos='-3e-05 0.001937 0.11407' mass='1.065387' fullinertia='0.006967 0.007003 0.000673 -1e-06 -1e-06 -1.5e-05' />
            <joint name='left-shoulder-yaw' type='hinge' axis='0 0 1' range='-100 100' stiffness='0' frictionloss='2' damping='2' armature='0.067899975' limited='true' pos='0 0 0'/>
            <geom type='mesh' mesh='left-arm-shoulder-yaw-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
            <geom type='cylinder' size='0.038 0.0675' pos='0 0 0.092558' euler='-0 0 -0' friction='0.7 0.01 0.005' contype='2' conaffinity='-3' mass='1.065387' rgba='0.5 0.5 0.5 0.0' class='collision'/>
            <geom type='cylinder' size='0.03 0.04' pos='0 0.0035 0.185' euler='90 -3.180554681e-15 22.50004688' friction='0.7 0.01 0.005' contype='2' conaffinity='-3' mass='1.065387' rgba='0.5 0.5 0.5 0.0' class='collision'/>
            <body name='left-elbow' pos='0 -0.0385 0.185' euler='90 -3.180554681e-15 22.50004688'>
              <inertial pos='0.107996 0.000521 -0.017765' mass='0.550582' fullinertia='0.000476 0.009564 0.009437 -2.9e-05 0.001403 9e-06' />
              <joint name='left-elbow' type='hinge' axis='0 0 1' range='-77.5 77.5' stiffness='0' frictionloss='2' damping='2' armature='0.173823936' limited='true' pos='0 0 0'/>
              <geom type='mesh' mesh='left-arm-elbow-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
              <geom type='cylinder' size='0.013 0.1565' pos='0.184 0.01 -0.026' euler='169.9999264 78.00000889 -179.9999524' friction='0.7 0.01 0.005' contype='2' conaffinity='-3' mass='0.550582' rgba='0.5 0.5 0.5 0.0' class='collision'/>
              <geom type='sphere' size='0.015' pos='0.37937 0 -0.061912' euler='-0 0 -0' friction='0.7 0.01 0.005' contype='2' conaffinity='-3' mass='0.550582' rgba='0.5 0.5 0.5 0.0' class='collision'/>
              <site name='left-hand' pos='0.369 0 -0.07912' euler='90 0 -10' size='0.01'/>
            </body>
          </body>
        </body>
      </body>
      <site name='lidar' pos='0.025 0 0.4925' euler='0 0 15' size='0.01'/>
      <site name='realsense-back' pos='-0.000607 0.025 -0.025283' euler='-180 45 180' size='0.01'/>
      <site name='realsense-down' pos='0.0305 0.025 -0.03268' euler='0 90 0' size='0.01'/>
      <site name='realsense-forward' pos='0.093981 0.0225 0.426449' euler='0 45 0' size='0.01'/>
      <site name='realsense-front' pos='0.061607 0.025 -0.025283' euler='0 45 0' size='0.01'/>
      <body name='right-hip-roll' pos='-0.001 -0.091 0' euler='-21.49994484 -90 0'>
        <inertial pos='-0.001967 -0.000244 0.031435' mass='0.915088' fullinertia='0.001017 0.001148 0.000766 3e-06 1.3e-05 4e-06' />
        <joint name='right-hip-roll' type='hinge' axis='0 0 1' range='-60 60' stiffness='0' frictionloss='1' damping='1' armature='0.173823936' limited='true' pos='0 0 0'/>
        <geom type='mesh' mesh='right-leg-hip-roll-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
        <body name='right-hip-yaw' pos='-0.0505 0 0.044' euler='0 -90 0'>
          <inertial pos='1e-05 0.001945 0.042033' mass='0.818753' fullinertia='0.001627 0.001929 0.00077 1e-06 2e-06 -5.3e-05' />
          <joint name='right-hip-yaw' type='hinge' axis='0 0 1' range='-40 40' stiffness='0' frictionloss='1' damping='1' armature='0.067899975' limited='true' pos='0 0 0'/>
          <geom type='mesh' mesh='right-leg-hip-yaw-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
          <body name='right-hip-pitch' pos='0 -0.004 0.068' euler='-90 -3.225761214e-14 -134.9999776'>
            <inertial pos='0.060537 -0.000521 -0.038857' mass='6.244279' fullinertia='0.011533 0.033345 0.033958 0.000171 0.000148 -0.000178' />
            <joint name='right-hip-pitch' type='hinge' axis='0 0 -1' range='-90 60' stiffness='0' frictionloss='0.5' damping='1' armature='0.1204731904' limited='true' pos='0 0 0'/>
            <geom type='mesh' mesh='right-leg-hip-pitch-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
            <geom type='cylinder' size='0.05 0.072' pos='0 0 -0.04' euler='-0 0 -0' friction='0.7 0.01 0.005' contype='4' conaffinity='-5' mass='6.244279' rgba='0.5 0.5 0.5 0.0' class='collision'/>
            <geom type='cylinder' size='0.05 0.072' pos='0.12 0 -0.04' euler='-0 0 -0' friction='0.7 0.01 0.005' contype='4' conaffinity='-5' mass='6.244279' rgba='0.5 0.5 0.5 0.0' class='collision'/>
            <geom type='box' size='0.06 0.05 0.072' pos='0.06 0 -0.04' euler='-0 0 -0' friction='0.7 0.01 0.005' contype='4' conaffinity='-5' mass='6.244279' rgba='0.5 0.5 0.5 0.0' class='collision'/>
            <site name='rhp_arod' pos='0 0 0.046' euler='-4.09 0.05 82.27' size='0.01'/>
            <body name='right-knee' pos='0.12 0 0.0045' euler='-0 0 90'>
              <inertial pos='0.045641 -0.042154 0.001657' mass='1.227077' fullinertia='0.002643 0.005098 0.007019 0.001832 6.6e-05 -4.5e-05' />
              <joint name='right-knee' type='hinge' axis='0 0 1' range='-58.4 80' stiffness='0' frictionloss='0.5' damping='1' armature='0.1204731904' limited='true' pos='0 0 0'/>
              <geom type='mesh' mesh='right-leg-knee-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
              <body name='right-shin' pos='0.060677 -0.047406 0' euler='-0 0 -0'>
                <!-- <inertial pos='0.174265 -0.010265 0.00107' mass='0.895793' fullinertia='0.001128 0.022492 0.022793 -0.001098 0.000196 3e-06' /> -->
                <inertial pos='0.17637939 0.01361614 0.00483379' mass='1.073941' fullinertia='0.00439118 0.02701227 0.03037536 -0.0007727 0.00046775 -0.00052522' />
                <joint name='right-shin' type='hinge' axis='0 0 1'  stiffness='6000' frictionloss='0' damping='0' armature='0' limited='false' pos='0 0 0'/>
                <geom type='mesh' mesh='right-leg-shin-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                <geom type='cylinder' size='0.03 0.1715' pos='0.23 0 0' euler='0 90 0' friction='0.7 0.01 0.005' contype='4' conaffinity='-5' mass='0.895793' rgba='0.5 0.5 0.5 0.0' class='collision'/>
                <geom type='cylinder' size='0.05 0.1' pos='0.125 -0.009 0' euler='-90 83.99999778 -9.128289303e-14' friction='0.7 0.01 0.005' contype='4' conaffinity='-5' mass='0.895793' rgba='0.5 0.5 0.5 0.0' class='collision'/>
                <body name='right-tarsus' pos='0.434759 -0.02 0' euler='-0 0 -102.9999562'>
                <!-- <inertial pos='0.100777 0.029183 0.000678' mass='1.322865' fullinertia='0.000932 0.016409 0.016501 -0.00061 0.000102 -9e-06' /> -->
                  <inertial pos='0.11692845 0.03641395 0.00050495' mass='1.493355' fullinertia='1.60070790e-03 2.14664590e-02 2.21038310e-02 -1.98205783e-03 8.97031761e-05 5.41167493e-06' />
                  <joint name='right-tarsus' type='hinge' axis='0 0 1' range='-71.6 50.3' stiffness='0' frictionloss='0' damping='0' armature='0' limited='true' pos='0 0 0'/>
                  <geom type='mesh' mesh='right-leg-tarsus-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                  <geom type='capsule' size='0.02 0.18' pos='0.21 0.03 0' euler='0 90 0' friction='0.7 0.01 0.005' contype='4' conaffinity='-5' mass='1.322865' rgba='0.5 0.5 0.5 0.0' class='collision'/>
                  <geom type='box' size='0.007 0.07 0.03' pos='-0.07 0.012 0' euler='-4.999998056 -4.600265621e-05 -64.9999639' friction='0.7 0.01 0.005' contype='4' conaffinity='-5' mass='1.322865' rgba='0.5 0.5 0.5 0.0' class='collision'/>
                  <body name='right-heel-spring' pos='-0.01766 0.029456 0.00104' euler='-4.470028555 0.3199786449 -155.799968'>
                    <inertial pos='0.049086 -0.004739 -4.5e-05' mass='0.230018' fullinertia='5.5e-05 0.00074 0.000701 -1.5e-05 1e-06 0' />
                    <joint name='right-heel-spring' type='hinge' axis='0 0 1' range='-6 6' stiffness='4375' frictionloss='0' damping='0' armature='0' limited='true' pos='0 0 0'/>
                    <geom type='mesh' mesh='right-leg-heel-spring-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                    <site name='rhs-arod' pos = '0.113789 0.011056 0' euler='0 0 0' size='0.01'/>
                  </body>
                  <body name='right-toe-A' pos='0.059 0.034 -0.0276' euler='-180 -7.015553385e-15 -88.9599851'>
                    <inertial pos='0.005161 -1e-06 -0.002248' mass='0.139557' fullinertia='2.9e-05 5.8e-05 7.4e-05 0 -4e-06 0' />
                    <joint name='right-toe-A' type='hinge' axis='0 0 1' range='-44.9815 46.2755' stiffness='0' frictionloss='1' damping='1' armature='0.036089474999999996' limited='true' pos='0 0 0'/>
                    <geom type='mesh' mesh='right-leg-toe-a-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                    <site name='rtA-rod' pos='0.057 0 -0.008' euler='-0.34 2.14 91.47' size='0.01'/>
                  </body>
                  <body name='right-toe-B' pos='0.111 0.034 0.0276' euler='-0 0 88.9599851'>
                    <inertial pos='0.005161 -1e-06 -0.002248' mass='0.139557' fullinertia='2.9e-05 5.8e-05 7.4e-05 0 -4e-06 0' />
                    <joint name='right-toe-B' type='hinge' axis='0 0 1' range='-45.5476 45.8918' stiffness='0' frictionloss='1' damping='1' armature='0.036089474999999996' limited='true' pos='0 0 0'/>
                    <geom type='mesh' mesh='right-leg-toe-b-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                    <site name='rtB-rod' pos='0.057 0 -0.008' euler='0.22 -3.08 -91.38' size='0.01'/>
                  </body>
                  <body name='right-toe-pitch' pos='0.408 0.04 0' euler='-0 0 -71.75004157'>
                    <inertial pos='-0.000141 -2e-06 3e-06' mass='0.043881' fullinertia='5e-06 8e-06 4e-06 0 0 0' />
                    <joint name='right-toe-pitch' type='hinge' axis='0 0 1' range='-34 44' stiffness='0' frictionloss='0' damping='0' armature='0' limited='true' pos='0 0 0'/>
                    <geom type='mesh' mesh='right-leg-toe-pitch-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                    <body name='right-toe-roll' pos='0 0 0' euler='0 90 0'>
                      <inertial pos='9e-06 0.028084 -0.023204' mass='0.531283' fullinertia='0.00187 0.001616 0.000843 0 0 -0.000566' />
                      <joint name='right-toe-roll' type='hinge' axis='0 0 1' range='-33 37' stiffness='0' frictionloss='0' damping='0' armature='0' limited='true' pos='0 0 0'/>
                      <geom type='mesh' mesh='right-leg-toe-roll-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
                      <geom type='box' size='0.04 0.1175 0.0115' pos='0 0.0437 -0.0255' euler='60.00002314 0 -0' friction='0.7 0.01 0.005' contype='4' conaffinity='-5' mass='0.531283' rgba='0.5 0.5 0.5 0.0' class='collision'/>
                      <site name='right-foot' pos='0 0.05456 -0.0315' euler='60 0 90' size='0.01'/>
                      <site name='rtrA-rod' pos='0.0179 0.009551 -0.054164' euler='0 0 0' size='0.01'/>
                      <site name='rtrB-rod' pos='-0.0181 0.009551 -0.054164' euler='0 0 0' size='0.01'/>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
      <body name='right-shoulder-roll' pos='-0.001 -0.12 0.4' euler='9.999933098 -90 0'>
        <inertial pos='-0.000819 0.003158 0.023405' mass='0.535396' fullinertia='0.000704 0.00075 0.000298 -1.4e-05 1.2e-05 -3.5e-05' />
        <joint name='right-shoulder-roll' type='hinge' axis='0 0 1' range='-75 75' stiffness='0' frictionloss='2' damping='2' armature='0.173823936' limited='true' pos='0 0 0'/>
        <geom type='mesh' mesh='right-arm-shoulder-roll-0' pos='0 0 0' euler='-0 0 15.99998964' material='robot'/>
        <body name='right-shoulder-pitch' pos='-0.00317 0.011055 0.0555' euler='-89.99999034 -16.0000415 44.99997193'>
          <inertial pos='-4.2e-05 0.061882 -0.073788' mass='1.440357' fullinertia='0.006761 0.002087 0.005778 6e-06 -3e-06 0.002046' />
          <joint name='right-shoulder-pitch' type='hinge' axis='0 0 -1' range='-145 145' stiffness='0' frictionloss='2' damping='2' armature='0.173823936' limited='true' pos='0 0 0'/>
          <geom type='mesh' mesh='right-arm-shoulder-pitch-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
          <geom type='cylinder' size='0.038 0.07' pos='0 0.093 -0.1' euler='-90 0 -0' friction='0.7 0.01 0.005' contype='8' conaffinity='-9' mass='1.440357' rgba='0.5 0.5 0.5 0.0' class='collision'/>
          <body name='right-shoulder-yaw' pos='0 0.165 -0.1' euler='-90 0 -0'>
            <inertial pos='-3e-05 -0.001937 0.11407' mass='1.065387' fullinertia='0.006967 0.007003 0.000673 1e-06 -1e-06 1.5e-05' />
            <joint name='right-shoulder-yaw' type='hinge' axis='0 0 1' range='-100 100' stiffness='0' frictionloss='2' damping='2' armature='0.067899975' limited='true' pos='0 0 0'/>
            <geom type='mesh' mesh='right-arm-shoulder-yaw-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
            <geom type='cylinder' size='0.038 0.0675' pos='0 0 0.092558' euler='-0 0 -0' friction='0.7 0.01 0.005' contype='8' conaffinity='-9' mass='1.065387' rgba='0.5 0.5 0.5 0.0' class='collision'/>
            <geom type='cylinder' size='0.03 0.04' pos='0 -0.0035 0.185' euler='-90 -3.360791161e-15 -22.50004688' friction='0.7 0.01 0.005' contype='8' conaffinity='-9' mass='1.065387' rgba='0.5 0.5 0.5 0.0' class='collision'/>
            <body name='right-elbow' pos='0 0.0385 0.185' euler='-90 -3.360791161e-15 -22.50004688'>
              <inertial pos='0.107996 -0.000521 -0.017765' mass='0.550582' fullinertia='0.000476 0.009564 0.009437 2.9e-05 0.001403 -9e-06' />
              <joint name='right-elbow' type='hinge' axis='0 0 1' range='-77.5 77.5' stiffness='0' frictionloss='2' damping='2' armature='0.173823936' limited='true' pos='0 0 0'/>
              <geom type='mesh' mesh='right-arm-elbow-0' pos='0 0 0' euler='-0 0 -0' material='robot'/>
              <geom type='cylinder' size='0.013 0.1565' pos='0.184 -0.01 -0.026' euler='-169.9999264 78.00000889 179.9999524' friction='0.7 0.01 0.005' contype='8' conaffinity='-9' mass='0.550582' rgba='0.5 0.5 0.5 0.0' class='collision'/>
              <geom type='sphere' size='0.015' pos='0.37937 0 -0.061912' euler='-0 0 -0' friction='0.7 0.01 0.005' contype='8' conaffinity='-9' mass='0.550582' rgba='0.5 0.5 0.5 0.0' class='collision'/>
              <site name='right-hand' pos='0.369 0 -0.07912' euler='-90 0 10' size='0.01'/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
*/

enum DigitBodyIdx : myint {
        world_id=-1,
        base_trans_id,
        base_rot_id,
        left_hip_roll_id,
        left_hip_yaw_id,
        left_hip_pitch_id,
        left_knee_id,
        left_shin_id,
        left_tarsus_id,
        left_heel_spring_id,
        left_toe_A_id,
        left_toe_B_id,
        left_toe_pitch_id,
        left_toe_roll_id,
        left_shoulder_roll_id,
        left_shoulder_pitch_id,
        left_shoulder_yaw_id,
        left_elbow_id,
        right_hip_roll_id,
        right_hip_yaw_id,
        right_hip_pitch_id,
        right_knee_id,
        right_shin_id,
        right_tarsus_id,
        right_heel_spring_id,
        right_toe_A_id,
        right_toe_B_id,
        right_toe_pitch_id,
        right_toe_roll_id,
        right_shoulder_roll_id,
        right_shoulder_pitch_id,
        right_shoulder_yaw_id,
        right_elbow_id
};


class DigitModel: public RigidBodyTree
{
    public:

    // constructors
    DigitModel() : RigidBodyTree(){

        // start constructing the model
        this->num_bodies = -1;
        this->num_sites = 0;

        // create the bodies
        //base_trans
        num_bodies++;
        RigidBody base_trans;
        base_trans.name = "base_trans";
        base_trans.parent = -1;
        base_trans.id = num_bodies; assert(base_trans.id == base_trans_id);
        base_trans.joint.name = "translational";
        base_trans.joint.joint_type = JointType::TRANSLATIONAL; num_q+=3; num_v+=3;
        base_trans.joint.parent_body = base_trans.id;
        base_trans.joint.limits.resize(3,2);
        base_trans.joint.limits << -MYINF, MYINF,
                            -MYINF, MYINF,
                            -MYINF, MYINF;

        base_trans.spI = SpatialInertia(0, Eigen::Matrix<myfloat,3,1>(0,0,0), Eigen::Matrix<myfloat,6,1>(0,0,0,0,0,0));
        base_trans.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 0.0), Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 0.0));

        //base_rot
        num_bodies++;
        RigidBody base_rot;
        base_rot.name = "base_rot";
        base_rot.parent = base_trans.id;
        base_rot.id = num_bodies; assert(base_rot.id == base_rot_id);
        base_rot.joint.name = "spherical"; 
        base_rot.joint.joint_type = JointType::SPHERICAL; num_q+=4; num_v+=3;
        base_rot.joint.parent_body = base_rot.id;
        base_rot.joint.limits.resize(4,2);
        base_rot.joint.limits << -1.0, 1.0,
                            -1.0, 1.0,
                            -1.0, 1.0,
                            -1.0, 1.0;

        base_rot.spI = SpatialInertia(15.028392, Eigen::Matrix<myfloat,3,1>(0.001636612541, 0.0002001180789, 0.2593064529), Eigen::Matrix<myfloat,6,1>(0.3759052548, 0.3441935, 0.09873232746, -8.776732849e-05, 0.008498611229, 6.621611757e-05));
        base_rot.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 0.0), Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 0.0));

        // left-hip-roll
        num_bodies++;
        RigidBody left_hip_roll;
        left_hip_roll.name = "left-hip-roll";
        left_hip_roll.parent = base_rot.id; 
        left_hip_roll.id = num_bodies; assert(left_hip_roll.id == left_hip_roll_id);
        left_hip_roll.joint.name = "left-hip-roll";
        left_hip_roll.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_hip_roll.joint.parent_body = left_hip_roll.id;
        left_hip_roll.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_hip_roll.joint.limits.resize(1,2);
        left_hip_roll.joint.limits = Eigen::Matrix<myfloat,1,2>(-60,60)*M_PI/180.0;
        left_hip_roll.joint.armature = 0.173823936; num_u+=1;
        left_hip_roll.joint.damping = 1.0;
        left_hip_roll.joint.frictionloss = 1.0;
        left_hip_roll.spI = SpatialInertia(0.915088, Eigen::Matrix<myfloat,3,1>(-0.001967, 0.000244, 0.031435), Eigen::Matrix<myfloat,6,1>(0.001017, 0.001148, 0.000766, -3e-06, 1.3e-05, -4e-06));
        left_hip_roll.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(21.49994484, -90, 0), Eigen::Matrix<myfloat,3,1>(-0.001, 0.091, 0));

        // left-hip-yaw
        num_bodies++;
        RigidBody left_hip_yaw;
        left_hip_yaw.name = "left-hip-yaw";
        left_hip_yaw.parent = left_hip_roll.id;
        left_hip_yaw.id = num_bodies; assert(left_hip_yaw.id == left_hip_yaw_id);
        left_hip_yaw.joint.name = "left-hip-yaw";
        left_hip_yaw.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_hip_yaw.joint.parent_body = left_hip_yaw.id;
        left_hip_yaw.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_hip_yaw.joint.limits.resize(1,2);
        left_hip_yaw.joint.limits = Eigen::Matrix<myfloat,1,2>(-40,40)*M_PI/180.0;
        left_hip_yaw.joint.armature = 0.067899975; num_u+=1;
        left_hip_yaw.joint.damping = 1.0;
        left_hip_yaw.joint.frictionloss = 1.0;

        left_hip_yaw.spI = SpatialInertia(0.818753, Eigen::Matrix<myfloat,3,1>(1e-05, -0.001945, 0.042033), Eigen::Matrix<myfloat,6,1>(0.001627, 0.001929, 0.00077, -1e-06, 2e-06, 5.3e-05));
        left_hip_yaw.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, -90.0, 0), Eigen::Matrix<myfloat,3,1>(-0.0505, 0, 0.044));

        // left-hip-pitch
        num_bodies++;
        RigidBody left_hip_pitch;
        left_hip_pitch.name = "left-hip-pitch";
        left_hip_pitch.parent = left_hip_yaw.id;
        left_hip_pitch.id = num_bodies; assert(left_hip_pitch.id == left_hip_pitch_id);
        left_hip_pitch.joint.name = "left-hip-pitch";
        left_hip_pitch.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_hip_pitch.joint.parent_body = left_hip_pitch.id;
        left_hip_pitch.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, -1.0);
        left_hip_pitch.joint.limits.resize(1,2);
        left_hip_pitch.joint.limits = Eigen::Matrix<myfloat,1,2>(-60,90)*M_PI/180.0;
        left_hip_pitch.joint.armature = 0.1204731904; num_u+=1;
        left_hip_pitch.joint.damping = 1.0;
        left_hip_pitch.joint.frictionloss = 0.5;

        left_hip_pitch.spI = SpatialInertia(6.244279, Eigen::Matrix<myfloat,3,1>(0.060537, 0.000521, -0.038857), Eigen::Matrix<myfloat,6,1>(0.011533, 0.033345, 0.033958, -0.000171, 0.000148, 0.000178));
        left_hip_pitch.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(90, 0, 135), Eigen::Matrix<myfloat,3,1>(0, 0.004, 0.068));

        // left-knee
        num_bodies++;
        RigidBody left_knee;
        left_knee.name = "left-knee";
        left_knee.parent = left_hip_pitch.id;
        left_knee.id = num_bodies; assert(left_knee.id == left_knee_id);
        left_knee.joint.name = "left-knee";
        left_knee.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_knee.joint.parent_body = left_knee.id;
        left_knee.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_knee.joint.limits.resize(1,2);
        left_knee.joint.limits = Eigen::Matrix<myfloat,1,2>(-80,58.4)*M_PI/180.0;
        left_knee.joint.armature = 0.1204731904; num_u+=1;
        left_knee.joint.damping = 1.0;
        left_knee.joint.frictionloss = 0.5;

        left_knee.spI = SpatialInertia(1.227077, Eigen::Matrix<myfloat,3,1>(0.045641, 0.042154, 0.001657), Eigen::Matrix<myfloat,6,1>(0.002643, 0.005098, 0.007019, -0.001832, 6.6e-05, 4.5e-05));
        left_knee.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, -90), Eigen::Matrix<myfloat,3,1>(0.12, 0, 0.0045));

        // left-shin
        num_bodies++;
        RigidBody left_shin;
        left_shin.name = "left-shin";
        left_shin.parent = left_knee.id;
        left_shin.id = num_bodies; assert(left_shin.id == left_shin_id);
        left_shin.joint.name = "left-shin";
        left_shin.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_shin.joint.parent_body = left_shin.id;
        left_shin.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_shin.joint.limits.resize(1,2);
        left_shin.joint.limits = Eigen::Matrix<myfloat,1,2>(-90,90)*M_PI/180.0;
        left_shin.joint.stiffness = 6000.0;

        left_shin.spI = SpatialInertia(1.073941, Eigen::Matrix<myfloat,3,1>(0.17637939, -0.01361614, 0.00483379), Eigen::Matrix<myfloat,6,1>(0.00439118, 0.02701227, 0.03037536, 0.0007727, 0.00046775, 0.00052522));
        left_shin.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, 0), Eigen::Matrix<myfloat,3,1>(0.060677, 0.047406, 0));

        // left-tarsus
        num_bodies++;
        RigidBody left_tarsus;
        left_tarsus.name = "left-tarsus";
        left_tarsus.parent = left_shin.id;
        left_tarsus.id = num_bodies; assert(left_tarsus.id == left_tarsus_id);
        left_tarsus.joint.name = "left-tarsus";
        left_tarsus.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_tarsus.joint.parent_body = left_tarsus.id;
        left_tarsus.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_tarsus.joint.limits.resize(1,2);
        left_tarsus.joint.limits = Eigen::Matrix<myfloat,1,2>(-50.3,71.6)*M_PI/180.0;

        left_tarsus.spI = SpatialInertia(1.493355, Eigen::Matrix<myfloat,3,1>(0.11692845, -0.03641395, 0.00050495), Eigen::Matrix<myfloat,6,1>(1.60070790e-03, 2.14664590e-02, 2.21038310e-02, 1.98205783e-03, 8.97031761e-05, -5.41167493e-06));
        left_tarsus.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, 103), Eigen::Matrix<myfloat,3,1>(0.434759, 0.02, 0));

        // left-heel-spring
        num_bodies++;
        RigidBody left_heel_spring;
        left_heel_spring.name = "left-heel-spring";
        left_heel_spring.parent = left_tarsus.id;
        left_heel_spring.id = num_bodies; assert(left_heel_spring.id == left_heel_spring_id);
        left_heel_spring.joint.name = "left-heel-spring";
        left_heel_spring.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_heel_spring.joint.parent_body = left_heel_spring.id;
        left_heel_spring.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_heel_spring.joint.limits.resize(1,2);
        left_heel_spring.joint.limits = Eigen::Matrix<myfloat,1,2>(-6,6)*M_PI/180.0;
        left_heel_spring.joint.stiffness = 4375.0;

        left_heel_spring.spI = SpatialInertia(0.230018, Eigen::Matrix<myfloat,3,1>(0.049086, 0.004739, -4.5e-05), Eigen::Matrix<myfloat,6,1>(5.5e-05, 0.00074, 0.000701, 1.5e-05, 1e-06, 0));
        left_heel_spring.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(4.470028555, 0.3199786449, 155.799968), Eigen::Matrix<myfloat,3,1>(-0.01766, -0.029456, 0.00104));

        // left-toe-A
        num_bodies++;
        RigidBody left_toe_A;
        left_toe_A.name = "left-toe-A";
        left_toe_A.parent = left_tarsus.id;
        left_toe_A.id = num_bodies; assert(left_toe_A.id == left_toe_A_id);
        left_toe_A.joint.name = "left-toe-A";
        left_toe_A.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_toe_A.joint.parent_body = left_toe_A.id;
        left_toe_A.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_toe_A.joint.limits.resize(1,2);
        left_toe_A.joint.limits = Eigen::Matrix<myfloat,1,2>(-46.2755,44.9815)*M_PI/180.0;
        left_toe_A.joint.armature = 0.036089474999999996; num_u+=1;
        left_toe_A.joint.damping = 1.0;
        left_toe_A.joint.frictionloss = 1.0;

        left_toe_A.spI = SpatialInertia(0.139557, Eigen::Matrix<myfloat,3,1>(0.005161, 1e-06, -0.002248), Eigen::Matrix<myfloat,6,1>(2.9e-05, 5.8e-05, 7.4e-05, 0, -4e-06, 0));
        left_toe_A.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(180, 0, 88.9599851), Eigen::Matrix<myfloat,3,1>(0.059, -0.034, -0.0276));

        // left-toe-B
        num_bodies++;
        RigidBody left_toe_B;
        left_toe_B.name = "left-toe-B";
        left_toe_B.parent = left_tarsus.id;
        left_toe_B.id = num_bodies; assert(left_toe_B.id == left_toe_B_id);
        left_toe_B.joint.name = "left-toe-B";
        left_toe_B.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_toe_B.joint.parent_body = left_toe_B.id;
        left_toe_B.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_toe_B.joint.limits.resize(1,2);
        left_toe_B.joint.limits = Eigen::Matrix<myfloat,1,2>(-45.8918,45.5476)*M_PI/180.0;
        left_toe_B.joint.armature = 0.036089474999999996; num_u+=1;
        left_toe_B.joint.damping = 1.0;
        left_toe_B.joint.frictionloss = 1.0;

        left_toe_B.spI = SpatialInertia(0.139557, Eigen::Matrix<myfloat,3,1>(0.005161, 1e-06, -0.002248), Eigen::Matrix<myfloat,6,1>(2.9e-05, 5.8e-05, 7.4e-05, 0, -4e-06, 0));
        left_toe_B.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, -88.9599851), Eigen::Matrix<myfloat,3,1>(0.111, -0.034, 0.0276));

        // left-toe-pitch
        num_bodies++;
        RigidBody left_toe_pitch;
        left_toe_pitch.name = "left-toe-pitch";
        left_toe_pitch.parent = left_tarsus.id;
        left_toe_pitch.id = num_bodies; assert(left_toe_pitch.id == left_toe_pitch_id);
        left_toe_pitch.joint.name = "left-toe-pitch";
        left_toe_pitch.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_toe_pitch.joint.parent_body = left_toe_pitch.id;
        left_toe_pitch.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_toe_pitch.joint.limits.resize(1,2);
        left_toe_pitch.joint.limits = Eigen::Matrix<myfloat,1,2>(-44,34)*M_PI/180.0;

        left_toe_pitch.spI = SpatialInertia(0.043881, Eigen::Matrix<myfloat,3,1>(-0.000141, 2e-06, 3e-06), Eigen::Matrix<myfloat,6,1>(5e-06, 8e-06, 4e-06, 0, 0, 0));
        left_toe_pitch.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, 71.75004157), Eigen::Matrix<myfloat,3,1>(0.408, -0.04, 0));

        // left-toe-roll
        num_bodies++;
        RigidBody left_toe_roll;
        left_toe_roll.name = "left-toe-roll";
        left_toe_roll.parent = left_toe_pitch.id;
        left_toe_roll.id = num_bodies; assert(left_toe_roll.id == left_toe_roll_id);
        left_toe_roll.joint.name = "left-toe-roll";
        left_toe_roll.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_toe_roll.joint.parent_body = left_toe_roll.id;
        left_toe_roll.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_toe_roll.joint.limits.resize(1,2);
        left_toe_roll.joint.limits = Eigen::Matrix<myfloat,1,2>(-37,33)*M_PI/180.0;

        left_toe_roll.spI = SpatialInertia(0.531283, Eigen::Matrix<myfloat,3,1>(9e-06, -0.028084, -0.023204), Eigen::Matrix<myfloat,6,1>(0.00187, 0.001616, 0.000843, 0, 0, 0.000566));
        left_toe_roll.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 90, 0), Eigen::Matrix<myfloat,3,1>(0, 0, 0));

        // left-shoulder-roll
        num_bodies++;
        RigidBody left_shoulder_roll;
        left_shoulder_roll.name = "left-shoulder-roll";
        left_shoulder_roll.parent = base_rot.id;
        left_shoulder_roll.id = num_bodies; assert(left_shoulder_roll.id == left_shoulder_roll_id);
        left_shoulder_roll.joint.name = "left-shoulder-roll";
        left_shoulder_roll.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_shoulder_roll.joint.parent_body = left_shoulder_roll.id;
        left_shoulder_roll.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_shoulder_roll.joint.limits.resize(1,2);
        left_shoulder_roll.joint.limits = Eigen::Matrix<myfloat,1,2>(-75,75)*M_PI/180.0;
        left_shoulder_roll.joint.armature = 0.173823936; num_u+=1;
        left_shoulder_roll.joint.damping = 2.0;
        left_shoulder_roll.joint.frictionloss = 2.0;

        left_shoulder_roll.spI = SpatialInertia(0.535396, Eigen::Matrix<myfloat,3,1>(-0.000819, -0.003158, 0.023405), Eigen::Matrix<myfloat,6,1>(0.000704, 0.00075, 0.000298, 1.4e-05, 1.2e-05, 3.5e-05));
        left_shoulder_roll.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-10.0, -90, 0), Eigen::Matrix<myfloat,3,1>(-0.001, 0.12, 0.4));

        // left-shoulder-pitch
        num_bodies++;
        RigidBody left_shoulder_pitch;
        left_shoulder_pitch.name = "left-shoulder-pitch";
        left_shoulder_pitch.parent = left_shoulder_roll.id;
        left_shoulder_pitch.id = num_bodies; assert(left_shoulder_pitch.id == left_shoulder_pitch_id);
        left_shoulder_pitch.joint.name = "left-shoulder-pitch";
        left_shoulder_pitch.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_shoulder_pitch.joint.parent_body = left_shoulder_pitch.id;
        left_shoulder_pitch.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, -1.0);
        left_shoulder_pitch.joint.limits.resize(1,2);
        left_shoulder_pitch.joint.limits = Eigen::Matrix<myfloat,1,2>(-145,145)*M_PI/180.0;
        left_shoulder_pitch.joint.armature = 0.173823936; num_u+=1;
        left_shoulder_pitch.joint.damping = 2.0;
        left_shoulder_pitch.joint.frictionloss = 2.0;

        left_shoulder_pitch.spI = SpatialInertia(1.440357, Eigen::Matrix<myfloat,3,1>(-4.2e-05, -0.061882, -0.073788), Eigen::Matrix<myfloat,6,1>(0.006761, 0.002087, 0.005778, -6e-06, -3e-06, -0.002046));
        left_shoulder_pitch.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(90, -16.0, -45), Eigen::Matrix<myfloat,3,1>(-0.00317, -0.011055, 0.0555));

        // left-shoulder-yaw
        num_bodies++;
        RigidBody left_shoulder_yaw;
        left_shoulder_yaw.name = "left-shoulder-yaw";
        left_shoulder_yaw.parent = left_shoulder_pitch.id;
        left_shoulder_yaw.id = num_bodies; assert(left_shoulder_yaw.id == left_shoulder_yaw_id);
        left_shoulder_yaw.joint.name = "left-shoulder-yaw";
        left_shoulder_yaw.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_shoulder_yaw.joint.parent_body = left_shoulder_yaw.id;
        left_shoulder_yaw.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_shoulder_yaw.joint.limits.resize(1,2);
        left_shoulder_yaw.joint.limits = Eigen::Matrix<myfloat,1,2>(-100,100)*M_PI/180.0;
        left_shoulder_yaw.joint.armature = 0.067899975; num_u+=1;
        left_shoulder_yaw.joint.damping = 2.0;
        left_shoulder_yaw.joint.frictionloss = 2.0;

        left_shoulder_yaw.spI = SpatialInertia(1.065387, Eigen::Matrix<myfloat,3,1>(-3e-05, 0.001937, 0.11407), Eigen::Matrix<myfloat,6,1>(0.006967, 0.007003, 0.000673, -1e-06, -1e-06, -1.5e-05));
        left_shoulder_yaw.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(90, 0, 0), Eigen::Matrix<myfloat,3,1>(0, -0.165, -0.1));

        // left-elbow
        num_bodies++;
        RigidBody left_elbow;
        left_elbow.name = "left-elbow";
        left_elbow.parent = left_shoulder_yaw.id;
        left_elbow.id = num_bodies; assert(left_elbow.id == left_elbow_id);
        left_elbow.joint.name = "left-elbow";
        left_elbow.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        left_elbow.joint.parent_body = left_elbow.id;
        left_elbow.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        left_elbow.joint.limits.resize(1,2);
        left_elbow.joint.limits = Eigen::Matrix<myfloat,1,2>(-77.5,77.5)*M_PI/180.0;
        left_elbow.joint.armature = 0.173823936; num_u+=1;
        left_elbow.joint.damping = 2.0;
        left_elbow.joint.frictionloss = 2.0;

        left_elbow.spI = SpatialInertia(0.550582, Eigen::Matrix<myfloat,3,1>(0.107996, 0.000521, -0.017765), Eigen::Matrix<myfloat,6,1>(0.000476, 0.009564, 0.009437, -2.9e-05, 0.001403, 9e-06));
        left_elbow.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(90, 0, 22.5), Eigen::Matrix<myfloat,3,1>(0, -0.0385, 0.185));

        // right-hip-roll
        num_bodies++;
        RigidBody right_hip_roll;
        right_hip_roll.name = "right-hip-roll";
        right_hip_roll.parent = base_rot.id;
        right_hip_roll.id = num_bodies; assert(right_hip_roll.id == right_hip_roll_id);
        right_hip_roll.joint.name = "right-hip-roll";
        right_hip_roll.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_hip_roll.joint.parent_body = right_hip_roll.id;
        right_hip_roll.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_hip_roll.joint.limits.resize(1,2);
        right_hip_roll.joint.limits = Eigen::Matrix<myfloat,1,2>(-60,60)*M_PI/180.0;
        right_hip_roll.joint.armature = 0.173823936; num_u+=1;
        right_hip_roll.joint.damping = 1.0;
        right_hip_roll.joint.frictionloss = 1.0;

        right_hip_roll.spI = SpatialInertia(0.915088, Eigen::Matrix<myfloat,3,1>(-0.001967, -0.000244, 0.031435), Eigen::Matrix<myfloat,6,1>(0.001017, 0.001148, 0.000766, 3e-06, 1.3e-05, 4e-06));
        right_hip_roll.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-21.49994484, -90, 0), Eigen::Matrix<myfloat,3,1>(-0.001, -0.091, 0));

        // right-hip-yaw
        num_bodies++;
        RigidBody right_hip_yaw;
        right_hip_yaw.name = "right-hip-yaw";
        right_hip_yaw.parent = right_hip_roll.id;
        right_hip_yaw.id = num_bodies; assert(right_hip_yaw.id == right_hip_yaw_id);
        right_hip_yaw.joint.name = "right-hip-yaw";
        right_hip_yaw.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_hip_yaw.joint.parent_body = right_hip_yaw.id;
        right_hip_yaw.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_hip_yaw.joint.limits.resize(1,2);
        right_hip_yaw.joint.limits = Eigen::Matrix<myfloat,1,2>(-40,40)*M_PI/180.0;
        right_hip_yaw.joint.armature = 0.067899975; num_u+=1;
        right_hip_yaw.joint.damping = 1.0;
        right_hip_yaw.joint.frictionloss = 1.0;

        right_hip_yaw.spI = SpatialInertia(0.818753, Eigen::Matrix<myfloat,3,1>(1e-05, 0.001945, 0.042033), Eigen::Matrix<myfloat,6,1>(0.001627, 0.001929, 0.00077, 1e-06, 2e-06, -5.3e-05));
        right_hip_yaw.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, -90.0, 0), Eigen::Matrix<myfloat,3,1>(-0.0505, 0, 0.044));

        // right-hip-pitch
        num_bodies++;
        RigidBody right_hip_pitch;
        right_hip_pitch.name = "right-hip-pitch";
        right_hip_pitch.parent = right_hip_yaw.id;
        right_hip_pitch.id = num_bodies; assert(right_hip_pitch.id == right_hip_pitch_id);
        right_hip_pitch.joint.name = "right-hip-pitch";
        right_hip_pitch.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_hip_pitch.joint.parent_body = right_hip_pitch.id;
        right_hip_pitch.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, -1.0);
        right_hip_pitch.joint.limits.resize(1,2);
        right_hip_pitch.joint.limits = Eigen::Matrix<myfloat,1,2>(-90,60)*M_PI/180.0;
        right_hip_pitch.joint.armature = 0.1204731904; num_u+=1;
        right_hip_pitch.joint.damping = 1.0;
        right_hip_pitch.joint.frictionloss = 0.5;

        right_hip_pitch.spI = SpatialInertia(6.244279, Eigen::Matrix<myfloat,3,1>(0.060537, -0.000521, -0.038857), Eigen::Matrix<myfloat,6,1>(0.011533, 0.033345, 0.033958, 0.000171, 0.000148, -0.000178));
        right_hip_pitch.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-90, 0, -135), Eigen::Matrix<myfloat,3,1>(0, -0.004, 0.068));

        // right-knee
        num_bodies++;
        RigidBody right_knee;
        right_knee.name = "right-knee";
        right_knee.parent = right_hip_pitch.id;
        right_knee.id = num_bodies; assert(right_knee.id == right_knee_id);
        right_knee.joint.name = "right-knee";
        right_knee.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_knee.joint.parent_body = right_knee.id;
        right_knee.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_knee.joint.limits.resize(1,2);
        right_knee.joint.limits = Eigen::Matrix<myfloat,1,2>(-58.4,80)*M_PI/180.0;
        right_knee.joint.armature = 0.1204731904; num_u+=1;
        right_knee.joint.damping = 1.0;
        right_knee.joint.frictionloss = 0.5;

        right_knee.spI = SpatialInertia(1.227077, Eigen::Matrix<myfloat,3,1>(0.045641, -0.042154, 0.001657), Eigen::Matrix<myfloat,6,1>(0.002643, 0.005098, 0.007019, 0.001832, 6.6e-05, -4.5e-05));
        right_knee.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, 90), Eigen::Matrix<myfloat,3,1>(0.12, 0, 0.0045));

        // right-shin
        num_bodies++;
        RigidBody right_shin;
        right_shin.name = "right-shin";
        right_shin.parent = right_knee.id;
        right_shin.id = num_bodies; assert(right_shin.id == right_shin_id);
        right_shin.joint.name = "right-shin";
        right_shin.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_shin.joint.parent_body = right_shin.id;
        right_shin.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_shin.joint.limits.resize(1,2);
        right_shin.joint.limits = Eigen::Matrix<myfloat,1,2>(-90,90)*M_PI/180.0;    
        right_shin.joint.stiffness = 6000.0;

        right_shin.spI = SpatialInertia(1.073941, Eigen::Matrix<myfloat,3,1>(0.17637939, 0.01361614, 0.00483379), Eigen::Matrix<myfloat,6,1>(0.00439118, 0.02701227, 0.03037536, -0.0007727, 0.00046775, -0.00052522));
        right_shin.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, 0), Eigen::Matrix<myfloat,3,1>(0.060677, -0.047406, 0));

        // right-tarsus
        num_bodies++;
        RigidBody right_tarsus;
        right_tarsus.name = "right-tarsus";
        right_tarsus.parent = right_shin.id;
        right_tarsus.id = num_bodies; assert(right_tarsus.id == right_tarsus_id);
        right_tarsus.joint.name = "right-tarsus";
        right_tarsus.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_tarsus.joint.parent_body = right_tarsus.id;
        right_tarsus.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_tarsus.joint.limits.resize(1,2);
        right_tarsus.joint.limits = Eigen::Matrix<myfloat,1,2>(-71.6,50.3)*M_PI/180.0;

        right_tarsus.spI = SpatialInertia(1.493355, Eigen::Matrix<myfloat,3,1>(0.11692845, 0.03641395, 0.00050495), Eigen::Matrix<myfloat,6,1>(1.60070790e-03, 2.14664590e-02, 2.21038310e-02, -1.98205783e-03, 8.97031761e-05, 5.41167493e-06));
        right_tarsus.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, -103), Eigen::Matrix<myfloat,3,1>(0.434759, -0.02, 0));

        // right-heel-spring
        num_bodies++;
        RigidBody right_heel_spring;
        right_heel_spring.name = "right-heel-spring";
        right_heel_spring.parent = right_tarsus.id;
        right_heel_spring.id = num_bodies; assert(right_heel_spring.id == right_heel_spring_id);
        right_heel_spring.joint.name = "right-heel-spring";
        right_heel_spring.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_heel_spring.joint.parent_body = right_heel_spring.id;
        right_heel_spring.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_heel_spring.joint.limits.resize(1,2);
        right_heel_spring.joint.limits = Eigen::Matrix<myfloat,1,2>(-6,6)*M_PI/180.0;
        right_heel_spring.joint.stiffness = 4375.0;

        right_heel_spring.spI = SpatialInertia(0.230018, Eigen::Matrix<myfloat,3,1>(0.049086, -0.004739, -4.5e-05), Eigen::Matrix<myfloat,6,1>(5.5e-05, 0.00074, 0.000701, -1.5e-05, 1e-06, 0));
        right_heel_spring.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-4.470028555, 0.3199786449, -155.799968), Eigen::Matrix<myfloat,3,1>(-0.01766, 0.029456, 0.00104));

        // right-toe-A
        num_bodies++;
        RigidBody right_toe_A;
        right_toe_A.name = "right-toe-A";
        right_toe_A.parent = right_tarsus.id;
        right_toe_A.id = num_bodies; assert(right_toe_A.id == right_toe_A_id);
        right_toe_A.joint.name = "right-toe-A";
        right_toe_A.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_toe_A.joint.parent_body = right_toe_A.id;
        right_toe_A.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_toe_A.joint.limits.resize(1,2);
        right_toe_A.joint.limits = Eigen::Matrix<myfloat,1,2>(-44.9815,46.2755)*M_PI/180.0;
        right_toe_A.joint.armature = 0.036089474999999996; num_u+=1;
        right_toe_A.joint.damping = 1.0;
        right_toe_A.joint.frictionloss = 1.0;

        right_toe_A.spI = SpatialInertia(0.139557, Eigen::Matrix<myfloat,3,1>(0.005161, -1e-06, -0.002248), Eigen::Matrix<myfloat,6,1>(2.9e-05, 5.8e-05, 7.4e-05, 0, -4e-06, 0));
        right_toe_A.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-180, 0, -88.9599851), Eigen::Matrix<myfloat,3,1>(0.059, 0.034, -0.0276));

        // right-toe-B
        num_bodies++;
        RigidBody right_toe_B;
        right_toe_B.name = "right-toe-B";
        right_toe_B.parent = right_tarsus.id;
        right_toe_B.id = num_bodies; assert(right_toe_B.id == right_toe_B_id);
        right_toe_B.joint.name = "right-toe-B";
        right_toe_B.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_toe_B.joint.parent_body = right_toe_B.id;
        right_toe_B.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_toe_B.joint.limits.resize(1,2);
        right_toe_B.joint.limits = Eigen::Matrix<myfloat,1,2>(-45.5476,45.8918)*M_PI/180.0;
        right_toe_B.joint.armature = 0.036089474999999996; num_u+=1;
        right_toe_B.joint.damping = 1.0;
        right_toe_B.joint.frictionloss = 1.0;

        right_toe_B.spI = SpatialInertia(0.139557, Eigen::Matrix<myfloat,3,1>(0.005161, -1e-06, -0.002248), Eigen::Matrix<myfloat,6,1>(2.9e-05, 5.8e-05, 7.4e-05, 0, -4e-06, 0));
        right_toe_B.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, 88.9599851), Eigen::Matrix<myfloat,3,1>(0.111, 0.034, 0.0276));

        // right-toe-pitch
        num_bodies++;
        RigidBody right_toe_pitch;
        right_toe_pitch.name = "right-toe-pitch";
        right_toe_pitch.parent = right_tarsus.id;
        right_toe_pitch.id = num_bodies; assert(right_toe_pitch.id == right_toe_pitch_id);
        right_toe_pitch.joint.name = "right-toe-pitch";
        right_toe_pitch.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_toe_pitch.joint.parent_body = right_toe_pitch.id;
        right_toe_pitch.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_toe_pitch.joint.limits.resize(1,2);
        right_toe_pitch.joint.limits = Eigen::Matrix<myfloat,1,2>(-34,44)*M_PI/180.0;

        right_toe_pitch.spI = SpatialInertia(0.043881, Eigen::Matrix<myfloat,3,1>(-0.000141, -2e-06, 3e-06), Eigen::Matrix<myfloat,6,1>(5e-06, 8e-06, 4e-06, 0, 0, 0));
        right_toe_pitch.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 0, -71.75004157), Eigen::Matrix<myfloat,3,1>(0.408, 0.04, 0));

        // right-toe-roll
        num_bodies++;
        RigidBody right_toe_roll;
        right_toe_roll.name = "right-toe-roll";
        right_toe_roll.parent = right_toe_pitch.id;
        right_toe_roll.id = num_bodies; assert(right_toe_roll.id == right_toe_roll_id);
        right_toe_roll.joint.name = "right-toe-roll";
        right_toe_roll.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_toe_roll.joint.parent_body = right_toe_roll.id;
        right_toe_roll.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_toe_roll.joint.limits.resize(1,2);
        right_toe_roll.joint.limits = Eigen::Matrix<myfloat,1,2>(-33,37)*M_PI/180.0;

        right_toe_roll.spI = SpatialInertia(0.531283, Eigen::Matrix<myfloat,3,1>(9e-06, 0.028084, -0.023204), Eigen::Matrix<myfloat,6,1>(0.00187, 0.001616, 0.000843, 0, 0, -0.000566));
        right_toe_roll.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, 90, 0), Eigen::Matrix<myfloat,3,1>(0, 0, 0));

        // right-shoulder-roll
        num_bodies++;
        RigidBody right_shoulder_roll;
        right_shoulder_roll.name = "right-shoulder-roll";
        right_shoulder_roll.parent = base_rot.id;
        right_shoulder_roll.id = num_bodies; assert(right_shoulder_roll.id == right_shoulder_roll_id);
        right_shoulder_roll.joint.name = "right-shoulder-roll";
        right_shoulder_roll.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_shoulder_roll.joint.parent_body = right_shoulder_roll.id;
        right_shoulder_roll.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_shoulder_roll.joint.limits.resize(1,2);
        right_shoulder_roll.joint.limits = Eigen::Matrix<myfloat,1,2>(-75,75)*M_PI/180.0;
        right_shoulder_roll.joint.armature = 0.173823936; num_u+=1;
        right_shoulder_roll.joint.damping = 2.0;
        right_shoulder_roll.joint.frictionloss = 2.0;

        right_shoulder_roll.spI = SpatialInertia(0.535396, Eigen::Matrix<myfloat,3,1>(-0.000819, 0.003158, 0.023405), Eigen::Matrix<myfloat,6,1>(0.000704, 0.00075, 0.000298, -1.4e-05, 1.2e-05, -3.5e-05));
        right_shoulder_roll.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(10.0, -90, 0), Eigen::Matrix<myfloat,3,1>(-0.001, -0.12, 0.4));

        // right-shoulder-pitch
        num_bodies++;
        RigidBody right_shoulder_pitch;
        right_shoulder_pitch.name = "right-shoulder-pitch";
        right_shoulder_pitch.parent = right_shoulder_roll.id;
        right_shoulder_pitch.id = num_bodies; assert(right_shoulder_pitch.id == right_shoulder_pitch_id);
        right_shoulder_pitch.joint.name = "right-shoulder-pitch";
        right_shoulder_pitch.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_shoulder_pitch.joint.parent_body = right_shoulder_pitch.id;
        right_shoulder_pitch.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, -1.0);
        right_shoulder_pitch.joint.limits.resize(1,2);
        right_shoulder_pitch.joint.limits = Eigen::Matrix<myfloat,1,2>(-145,145)*M_PI/180.0;
        right_shoulder_pitch.joint.armature = 0.173823936; num_u+=1;
        right_shoulder_pitch.joint.damping = 2.0;
        right_shoulder_pitch.joint.frictionloss = 2.0;

        right_shoulder_pitch.spI = SpatialInertia(1.440357, Eigen::Matrix<myfloat,3,1>(-4.2e-05, 0.061882, -0.073788), Eigen::Matrix<myfloat,6,1>(0.006761, 0.002087, 0.005778, 6e-06, -3e-06, 0.002046));
        right_shoulder_pitch.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-90, -16.0, 45), Eigen::Matrix<myfloat,3,1>(-0.00317, 0.011055, 0.0555));

        // right-shoulder-yaw
        num_bodies++;
        RigidBody right_shoulder_yaw;
        right_shoulder_yaw.name = "right-shoulder-yaw";
        right_shoulder_yaw.parent = right_shoulder_pitch.id;
        right_shoulder_yaw.id = num_bodies; assert(right_shoulder_yaw.id == right_shoulder_yaw_id);
        right_shoulder_yaw.joint.name = "right-shoulder-yaw";
        right_shoulder_yaw.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_shoulder_yaw.joint.parent_body = right_shoulder_yaw.id;   
        right_shoulder_yaw.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_shoulder_yaw.joint.limits.resize(1,2);
        right_shoulder_yaw.joint.limits = Eigen::Matrix<myfloat,1,2>(-100,100)*M_PI/180.0;
        right_shoulder_yaw.joint.armature = 0.067899975; num_u+=1;
        right_shoulder_yaw.joint.damping = 2.0;
        right_shoulder_yaw.joint.frictionloss = 2.0;

        right_shoulder_yaw.spI = SpatialInertia(1.065387, Eigen::Matrix<myfloat,3,1>(-3e-05, -0.001937, 0.11407), Eigen::Matrix<myfloat,6,1>(0.006967, 0.007003, 0.000673, 1e-06, -1e-06, 1.5e-05));
        right_shoulder_yaw.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-90, 0, 0), Eigen::Matrix<myfloat,3,1>(0, 0.165, -0.1));
        
        // right-elbow
        num_bodies++;
        RigidBody right_elbow;
        right_elbow.name = "right-elbow";
        right_elbow.parent = right_shoulder_yaw.id;
        right_elbow.id = num_bodies; assert(right_elbow.id == right_elbow_id);
        right_elbow.joint.name = "right-elbow";
        right_elbow.joint.joint_type = JointType::REVOLUTE; num_q+=1; num_v+=1;
        right_elbow.joint.parent_body = right_elbow.id;
        right_elbow.joint.joint_axis = Eigen::Matrix<myfloat,3,1>(0.0, 0.0, 1.0);
        right_elbow.joint.limits.resize(1,2);
        right_elbow.joint.limits = Eigen::Matrix<myfloat,1,2>(-77.5,77.5)*M_PI/180.0;
        right_elbow.joint.armature = 0.173823936; num_u+=1;
        right_elbow.joint.damping = 2.0;
        right_elbow.joint.frictionloss = 2.0;

        right_elbow.spI = SpatialInertia(0.550582, Eigen::Matrix<myfloat,3,1>(0.107996, -0.000521, -0.017765), Eigen::Matrix<myfloat,6,1>(0.000476, 0.009564, 0.009437, 2.9e-05, 0.001403, -9e-06));
        right_elbow.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(-90, 0, -22.5), Eigen::Matrix<myfloat,3,1>(0, 0.0385, 0.185));

        num_bodies++; // final

        // Add sites
        // // base
        Site imu;
        imu.name = "imu";
        imu.parent_body = base_rot.id;
        imu.Xtree = PluckerTransform(Eigen::Matrix<myfloat,3,1>(0, -90, 0), Eigen::Matrix<myfloat,3,1>(0, 0, 0.0));



        sites.push_back(imu);

        // push all bodies into a vector
        bodies.push_back(base_trans);
        bodies.push_back(base_rot);
        bodies.push_back(left_hip_roll);
        bodies.push_back(left_hip_yaw);
        bodies.push_back(left_hip_pitch);
        bodies.push_back(left_knee);
        bodies.push_back(left_shin);
        bodies.push_back(left_tarsus);
        bodies.push_back(left_heel_spring);
        bodies.push_back(left_toe_A);
        bodies.push_back(left_toe_B);
        bodies.push_back(left_toe_pitch);
        bodies.push_back(left_toe_roll);
        bodies.push_back(left_shoulder_roll);
        bodies.push_back(left_shoulder_pitch);
        bodies.push_back(left_shoulder_yaw);
        bodies.push_back(left_elbow);
        bodies.push_back(right_hip_roll);
        bodies.push_back(right_hip_yaw);
        bodies.push_back(right_hip_pitch);
        bodies.push_back(right_knee);
        bodies.push_back(right_shin);
        bodies.push_back(right_tarsus);
        bodies.push_back(right_heel_spring);
        bodies.push_back(right_toe_A);
        bodies.push_back(right_toe_B);
        bodies.push_back(right_toe_pitch);
        bodies.push_back(right_toe_roll);
        bodies.push_back(right_shoulder_roll);
        bodies.push_back(right_shoulder_pitch);
        bodies.push_back(right_shoulder_yaw);
        bodies.push_back(right_elbow);

        // assert id matches index
        for (int i = 0; i < bodies.size(); i++) {
            if(bodies[i].id != i){
                std::cerr<<"id does not match index"<<std::endl;
                std::cerr<<"id: "<<bodies[i].id<<std::endl;
                std::cerr<<"index: "<<i<<std::endl;
            }
        }

        // add joint states
        for (int i = 0; i < num_bodies; i++) {
            JointType jt = bodies[i].joint.joint_type;
            switch (jt) {
                case JointType::REVOLUTE:
                    bodies[i].joint.q.resize(1); bodies[i].joint.q.setZero();
                    bodies[i].joint.v.resize(1); bodies[i].joint.v.setZero();
                    bodies[i].calculate_Xjtree();
                    break;
                case JointType::TRANSLATIONAL:
                    bodies[i].joint.q.resize(3); bodies[i].joint.q.setZero();
                    bodies[i].joint.v.resize(3); bodies[i].joint.v.setZero();
                    bodies[i].calculate_Xjtree();
                    break;
                case JointType::SPHERICAL:
                    bodies[i].joint.q.resize(4); bodies[i].joint.q.setZero(); bodies[i].joint.q(0) = 1.0;
                    bodies[i].joint.v.resize(3); bodies[i].joint.v.setZero();
                    bodies[i].calculate_Xjtree();
                    break;
                default:
                    std::cerr<<"joint type not implemented"<<std::endl;
                    assert(false);
                    break;
            }
            // std::cout << bodies[i]<<std::endl;
        }

        // tests
        std::cout<<"num_bodies: "<<num_bodies<<std::endl;
        std::cout<<"bodies.size(): "<<bodies.size()<<std::endl;
        std::cout<<"num_q: "<<num_q<<std::endl;
        std::cout<<"num_v: "<<num_v<<std::endl;
        std::cout<<"num_u: "<<num_u<<std::endl;
        
        std::cout<<"FwdKinematics"<<std::endl;
        bodies[0].set_pos(Eigen::Matrix<myfloat,3,1>(0,0,100));
        bodies[1].set_pos(Eigen::Matrix<myfloat,4,1>(0.9387913,0.2397128, 0.2397128, 0.0612087));
        bodies[left_hip_pitch_id].set_pos(Eigen::Matrix<myfloat,1,1>(0.1));
        std::cout<<this->forward_kinematics( Pose(Eigen::Matrix<myfloat,3,1>({-60,0,-90}),Eigen::Matrix<myfloat,3,1>({0,-0.05456,-0.0315})),left_toe_roll_id);

        std::cout<<"Body Jacobian"<<std::endl;
        std::cout<<this->spatial_body_jacobian(base_rot_id,0)<<std::endl;
        // this->print_state();




    }

    void print_state() {
        std::cout<<"q: "<<std::endl;
        for (int i = 0; i < bodies.size(); i++) {
            std::cout<<bodies[i].joint.q<<std::endl;
        }
        std::cout<<"v: "<<std::endl;
        for (int i = 0; i < bodies.size(); i++) {
            std::cout<<bodies[i].joint.v<<std::endl;
        }
    }

/**
 * @brief Given a string of the form "a b c", returns a vector of the form [a, b, c] where a, b, c are floats
 * 
 * @param str Input string
 * @return Eigen::Matrix<myfloat, -1, 1> Output vector
 */
Eigen::Matrix<myfloat, -1, 1> str2vec(std::string str){
    std::stringstream ss(str);
    Eigen::Matrix<myfloat, -1, 1> vec;
    myfloat val;
    myint i = 0;
    while(ss >> val){
        // add dimension to the vector
        vec.conservativeResize(i+1);
        vec(i) = val;
        i++;
    }
    return vec;
}

/**
 * @brief myfloat from string
 * 
 * @param std::string
 * @return myfloat 
 */
myfloat str2f(std::string str){
    std::stringstream ss(str);
    myfloat val;
    ss >> val;
    return val;
}
};
#endif // DIGIT_MODEL_HXX
