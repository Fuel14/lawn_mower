import 'dart:async';
import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_blue_classic/flutter_blue_classic.dart';
import 'package:lawn_mower_app_ver2/data/notifiers.dart';

class BluetoothPage extends StatefulWidget {
  const BluetoothPage({super.key, required this.connection});
  final BluetoothConnection connection;

  @override
  State<BluetoothPage> createState() => _BluetoothPageState();
}

class _BluetoothPageState extends State<BluetoothPage> {
  StreamSubscription? _readSubscription;
  final List<String> _receivedInput = [];

  @override
  void initState() {
    if (subscribtionStateNotifier.value == false) {
      _readSubscription = widget.connection.input?.listen((event) {
        if (mounted) {
          setState(() => _receivedInput.add(utf8.decode(event)));
        }
      });
      readSubscriptionNotifier.value = _readSubscription;
      subscribtionStateNotifier.value = true;
    }
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return ListView(
      children: [
        ElevatedButton(
          onPressed: () {
            try {
              widget.connection.writeString("Kuay");
            } catch (e) {
              if (kDebugMode) print(e);
              ScaffoldMessenger.maybeOf(context)?.showSnackBar(
                SnackBar(
                  content: Text(
                    "Error sending to device. Device is ${widget.connection.isConnected ? "connected" : "not connected"}",
                  ),
                ),
              );
            }
          },
          child: const Text("Send hello world to remote device"),
        ),
        const Divider(),
        Padding(
          padding: const EdgeInsets.symmetric(vertical: 8),
          child: Text(
            "Received data",
            style: Theme.of(context).textTheme.titleLarge,
          ),
        ),
        for (String input in _receivedInput) Text(input),
      ],
    );
  }
}
