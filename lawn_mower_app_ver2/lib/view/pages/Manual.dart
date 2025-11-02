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
            SizedBox(height: 50.0),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                FilledButton(onPressed: () {widget.connection.writeString("S7E");}, child: Text('Start Auto Driving')),
              ],
            ),
            SizedBox(height: 20.0),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                FilledButton(onPressed: () {widget.connection.writeString("S1E");}, child: Text('Play Sound 1')),
                FilledButton(onPressed: () {widget.connection.writeString("S2E");}, child: Text('Play Sound 2')),
              ],
            ),
            SizedBox(height: 20.0),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                FilledButton(onPressed: () {widget.connection.writeString("S3E");}, child: Text('Play Sound 3')),
                FilledButton(onPressed: () {widget.connection.writeString("S4E");}, child: Text('Play Sound 4')),
              ],
            ),
            SizedBox(height: 20.0),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                FilledButton(onPressed: () {widget.connection.writeString("S5E");}, child: Text('LED 1')),
                FilledButton(onPressed: () {widget.connection.writeString("S6E");}, child: Text('LED 2')),
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
                widget.connection.writeString("x${(details.x*255).round()}y${(details.y*255).round()}E");
              },
              period: Duration(milliseconds: 200),
            ),
          ],
        ),
      ),
    );
  }
}
