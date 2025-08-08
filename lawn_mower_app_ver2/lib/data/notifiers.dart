//ValueNotifier: store data
//ValueListenalbeBuilder: receiver data

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_blue_classic/flutter_blue_classic.dart';

ValueNotifier <int> selectPageNotifier = ValueNotifier(0);
ValueNotifier <bool> screenLightStateNotifier = ValueNotifier(false);
ValueNotifier <bool> bluetoothDeviceConnectingStatusNotifier = ValueNotifier(false);
ValueNotifier <BluetoothConnection?> bluetoothDeviceAdressNotifier = ValueNotifier(null) ;
ValueNotifier <BluetoothDevice?> bluetoothDeviceDataNotifier = ValueNotifier(null);
ValueNotifier <StreamSubscription?> readSubscriptionNotifier = ValueNotifier(null);
ValueNotifier <bool> subscribtionStateNotifier = ValueNotifier(false);