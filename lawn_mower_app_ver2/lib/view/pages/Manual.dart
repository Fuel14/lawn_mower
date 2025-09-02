import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter_blue_classic/flutter_blue_classic.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:lawn_mower_app_ver2/data/notifiers.dart';

class ManualPage extends StatefulWidget {
  const ManualPage({super.key, required this.connection});

  final BluetoothConnection connection;
  @override
  State<ManualPage> createState() => _ManualPageState();
}

class _ManualPageState extends State<ManualPage> {
  @override
  Uint8List bt_data= Uint8List(4);
  Uint8List bt_switch_data= Uint8List(1);
  @override
  Widget build(BuildContext context) {
    return SingleChildScrollView(
      child: Padding(
        padding: const EdgeInsets.all(8.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            SizedBox(height: 100.0),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                FilledButton(onPressed: () {bt_switch_data = Uint8List.fromList([5]); widget.connection.output.add(bt_switch_data);}, child: Text('Play Sound 1')),
                FilledButton(onPressed: () {bt_switch_data = Uint8List.fromList([6]); widget.connection.output.add(bt_switch_data);}, child: Text('Play Sound 2')),
              ],
            ),
            SizedBox(height: 20.0),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                FilledButton(onPressed: () {bt_switch_data = Uint8List.fromList([7]); widget.connection.output.add(bt_switch_data);}, child: Text('Play Sound 3')),
                FilledButton(onPressed: () {bt_switch_data = Uint8List.fromList([8]); widget.connection.output.add(bt_switch_data);}, child: Text('Play Sound 4')),
              ],
            ),
            SizedBox(height: 20.0),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                FilledButton(onPressed: () {bt_switch_data = Uint8List.fromList([9]); widget.connection.output.add(bt_switch_data);}, child: Text('LED 1')),
                FilledButton(onPressed: () {bt_switch_data = Uint8List.fromList([10]); widget.connection.output.add(bt_switch_data);}, child: Text('LED 2')),
              ],
            ),
            SizedBox(height: 100.0),
            Joystick(
              base: JoystickBase(
                decoration: JoystickBaseDecoration(
                  color: Colors.black,
                  drawOuterCircle: false,
                ),
                arrowsDecoration: JoystickArrowsDecoration(color: Colors.blue),
              ),
              listener: (details) {
                if(details.x >0 && details.y <0){
                  bt_data = Uint8List.fromList([1,(details.x.abs()*255).round(),(details.y.abs()*255).round(),69]);   
                }
                else if(details.x <0 && details.y <0){
                  bt_data = Uint8List.fromList([2,(details.x.abs()*255).round(),(details.y.abs()*255).round(),69]);   
                }
                else if(details.x <0 && details.y >0){
                  bt_data = Uint8List.fromList([3,(details.x.abs()*255).round(),(details.y.abs()*255).round(),69]);   
                }
                else if(details.x >0 && details.y >0){
                  bt_data = Uint8List.fromList([4,(details.x.abs()*255).round(),(details.y.abs()*255).round(),69]);   
                }
                else if(details.x ==0 && details.y ==0){
                  bt_data = Uint8List.fromList([0,(details.x.abs()*255).round(),(details.y.abs()*255).round(),69]);   
                }
                widget.connection.output.add(bt_data);
              },
              period: Duration(milliseconds: 200),
            ),
          ],
        ),
      ),
    );
  }
}
