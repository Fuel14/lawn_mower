import 'dart:async';
import 'dart:convert';
import 'dart:math';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_blue_classic/flutter_blue_classic.dart';
import 'package:lawn_mower_app_ver2/data/constants.dart';
import 'package:lawn_mower_app_ver2/data/notifiers.dart';
import 'package:lawn_mower_app_ver2/view/pages/GPS.dart';
import 'package:lawn_mower_app_ver2/view/pages/Manual.dart';
import 'package:lawn_mower_app_ver2/view/pages/bluetooth_page.dart';
import 'package:lawn_mower_app_ver2/view/pages/home_page.dart';
import 'package:lawn_mower_app_ver2/view/pages/no_bt_page.dart';
import 'package:lawn_mower_app_ver2/view/pages/setting_page.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'widget/navbar_widget.dart';

class WidgetTree extends StatefulWidget {
  const WidgetTree({super.key});

  @override
  State<WidgetTree> createState() => _WidgetTreeState();
}

class _WidgetTreeState extends State<WidgetTree> {
  StreamSubscription? _readSubscription;
  final List<String> _receivedInput = [];
  late List<String> _messageChunks = [];

  int _currentChunkIndex = 0;
  bool _isSending = false;
  bool _messageComplete = false;
  Timer? _responseTimer;
  String? _currentMessageId;
  final int _responseTimeoutSeconds = 10; // 10 seconds timeout

  // ADD THESE NEW VARIABLES FOR COMPLETION MESSAGES
  bool _showCompletionMessage = false;
  String _completionStatus = '';
  int _totalChunks = 0;
  int _lastAcknowledgedChunk = -1;

  get _flutterBlueClassicPlugin => FlutterBlueClassic();

  @override
  Widget build(BuildContext context) {
    return SafeArea(
      child: ValueListenableBuilder(
        valueListenable: readSubscriptionNotifier,
        builder: (context, readSubscription, child) {
          return ValueListenableBuilder(
            valueListenable: bluetoothDeviceAdressNotifier,
            builder: (context, connection, child) {
              return Scaffold(
                appBar: AppBar(
                  title: Text('UrMaMa'),
                  centerTitle: true,
                  actions: [
                    // Light Dark Mode switch Icon
                    IconButton(
                      onPressed: () async {
                        screenLightStateNotifier.value =
                            !screenLightStateNotifier.value;
                        final SharedPreferences pref =
                            await SharedPreferences.getInstance();
                        await pref.setBool(
                          KConstant.isLight,
                          screenLightStateNotifier.value,
                        );
                      },
                      icon: ValueListenableBuilder(
                        valueListenable: screenLightStateNotifier,
                        builder: (context, screenlightstate, child) {
                          return Icon(
                            screenlightstate == true
                                ? Icons.dark_mode
                                : Icons.light_mode,
                          );
                        },
                      ),
                    ),

                    // To setting page icon
                    IconButton(
                      onPressed: () {
                        Navigator.push(
                          context,
                          MaterialPageRoute(
                            builder: (context) {
                              return SettingPage();
                            },
                          ),
                        );
                      },
                      icon: Icon(Icons.settings),
                    ),
                    ValueListenableBuilder(
                      valueListenable: bluetoothDeviceDataNotifier,
                      builder: (context, result, child) {
                        return IconButton(
                          onPressed: () async {
                            if (connection == null) {
                              ScaffoldMessenger.maybeOf(context)?.showSnackBar(
                                const SnackBar(
                                  content: Text("No selecting device"),
                                  duration: Duration(seconds: 1),
                                ),
                              );
                            } else {
                              bluetoothDeviceConnectingStatusNotifier.value =
                                  !connection!.isConnected;
                              if (connection?.isConnected == false) {
                                try {
                                  connection = await _flutterBlueClassicPlugin
                                      .connect(result?.address);
                                  bluetoothDeviceAdressNotifier.value =
                                      connection;
                                } catch (e) {
                                  if (kDebugMode) print(e);
                                  connection?.dispose();
                                  ScaffoldMessenger.maybeOf(
                                    context,
                                  )?.showSnackBar(
                                    const SnackBar(
                                      content: Text(
                                        "Error connecting to device",
                                      ),
                                      duration: Duration(seconds: 1),
                                    ),
                                  );
                                }
                              } else {
                                try {
                                  connection?.dispose();
                                  readSubscription?.cancel();
                                  bluetoothDeviceAdressNotifier.value =
                                      connection;
                                  bluetoothDeviceDataNotifier.value = result;
                                  subscribtionStateNotifier.value = false;
                                } catch (e) {
                                  if (kDebugMode) print(e);
                                  connection?.dispose();
                                  ScaffoldMessenger.maybeOf(
                                    context,
                                  )?.showSnackBar(
                                    const SnackBar(
                                      content: Text(
                                        "Error disconnect to device",
                                      ),
                                    ),
                                  );
                                }
                              }
                            }
                          },
                          icon: ValueListenableBuilder(
                            valueListenable:
                                bluetoothDeviceConnectingStatusNotifier,
                            builder:
                                (context, bluetoothConnetionStatus, child) {
                                  return Icon(
                                    bluetoothConnetionStatus == true
                                        ? Icons.bluetooth_connected
                                        : Icons.bluetooth_disabled,
                                    color: bluetoothConnetionStatus == true
                                        ? Colors.blue
                                        : Colors.grey,
                                  );
                                },
                          ),
                        );
                      },
                    ),
                    ValueListenableBuilder(
                      valueListenable: gpsDataNotifier,
                      builder: (context, gpsData, child) {
                        return TextButton(
                          onPressed: () {
                            if (connection == null) {
                              ScaffoldMessenger.of(context).showSnackBar(
                                const SnackBar(
                                  content: Text("No device selected"),
                                  duration: Duration(seconds: 1),
                                ),
                              );
                            } else if (connection!.isConnected == false) {
                              ScaffoldMessenger.of(context).showSnackBar(
                                const SnackBar(
                                  content: Text("No device connection"),
                                  duration: Duration(seconds: 1),
                                ),
                              );
                            } else {
                              sent_GPS_data(connection!, gpsData);
                            }
                          },
                          child: Column(
                            children: [Icon(Icons.upload), Text("Upload GPS")],
                          ),
                        );
                      },
                    ),
                  ],
                ),

                //Body pages
                body: Stack(
                  children: [
                    ValueListenableBuilder(
                      valueListenable: selectPageNotifier,
                      builder: (context, selectedPage, child) {
                        return _buildPage(selectedPage);
                      },
                    ),
                    
                    // ADD COMPLETION MESSAGE OVERLAY
                    Positioned(
                      top: 10,
                      left: 16,
                      right: 16,
                      child: _buildCompletionMessage(),
                    ),
                    
                    // ADD UPLOAD PROGRESS OVERLAY
                    Positioned(
                      top: 70,
                      left: 16,
                      right: 16,
                      child: _buildUploadProgress(),
                    ),
                  ],
                ),

                //Bottom Navigation
                bottomNavigationBar: NavbarWidget(),
              );
            },
          );
        },
      ),
    );
  }

  // ADD COMPLETION MESSAGE WIDGET
  Widget _buildCompletionMessage() {
    if (!_showCompletionMessage) return SizedBox.shrink();
    
    return Container(
      padding: EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: _completionStatus.contains('success') 
            ? Colors.green.withOpacity(0.9) 
            : Colors.red.withOpacity(0.9),
        borderRadius: BorderRadius.circular(12),
        boxShadow: [
          BoxShadow(
            color: Colors.black26,
            blurRadius: 8,
            offset: Offset(0, 2),
          ),
        ],
      ),
      child: Row(
        children: [
          Icon(
            _completionStatus.contains('success') 
                ? Icons.check_circle 
                : Icons.error,
            color: Colors.white,
            size: 24,
          ),
          SizedBox(width: 12),
          Expanded(
            child: Text(
              _completionStatus,
              style: TextStyle(
                color: Colors.white,
                fontWeight: FontWeight.bold,
                fontSize: 16,
              ),
            ),
          ),
          IconButton(
            icon: Icon(Icons.close, color: Colors.white),
            onPressed: () {
              setState(() {
                _showCompletionMessage = false;
              });
            },
          ),
        ],
      ),
    );
  }

  // ADD UPLOAD PROGRESS WIDGET
  Widget _buildUploadProgress() {
    if (!_isSending) return SizedBox.shrink();
    
    return Container(
      padding: EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: Colors.blue.withOpacity(0.9),
        borderRadius: BorderRadius.circular(12),
      ),
      child: Column(
        children: [
          Row(
            children: [
              Icon(Icons.bluetooth_searching, color: Colors.white, size: 20),
              SizedBox(width: 8),
              Text(
                'Uploading to ESP32...',
                style: TextStyle(color: Colors.white, fontWeight: FontWeight.bold),
              ),
            ],
          ),
          SizedBox(height: 8),
          LinearProgressIndicator(
            value: _totalChunks > 0 ? _currentChunkIndex / _totalChunks : 0,
            backgroundColor: Colors.white.withOpacity(0.3),
            valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
          ),
          SizedBox(height: 4),
          Text(
            'Chunk $_currentChunkIndex/$_totalChunks',
            style: TextStyle(color: Colors.white, fontSize: 12),
          ),
          if (_lastAcknowledgedChunk >= 0)
            Text(
              'Last ACK: ${_lastAcknowledgedChunk + 1}',
              style: TextStyle(color: Colors.white, fontSize: 10),
            ),
        ],
      ),
    );
  }

  Widget _buildPage(int selectedPage) {
    switch (selectedPage) {
      case 0:
        return const HomePage();
      case 1:
        return ValueListenableBuilder(
          valueListenable: bluetoothDeviceAdressNotifier,
          builder: (context, connection, child) {
            if (connection == null) {
              return NoBtPage();
            } else {
              return BluetoothPage(connection: connection);
            }
          },
        );
      case 2:
        return ValueListenableBuilder(
          valueListenable: bluetoothDeviceAdressNotifier,
          builder: (context, connection, child) {
            if (connection == null) {
              return NoBtPage();
            } else {
              return ManualPage(connection: connection);
            }
          },
        );

      case 3:
        return gpsPage();
      default:
        return const HomePage(); // Fallback
    }
  }

  void sent_GPS_data(BluetoothConnection connection, String gpsData) {
    if (subscribtionStateNotifier.value == false) {
      _readSubscription = connection.input?.listen((event) {
        if (mounted) {
          setState(() {
            final message = utf8.decode(event);
            _receivedInput.add(message);
          });
        }
      });
      readSubscriptionNotifier.value = _readSubscription;
      subscribtionStateNotifier.value = true;
    }
    
    // Use the enhanced chunking function
    final chunks = chunkMessageWithHeaders(gpsData);
    _sendChunksToEsp32(chunks, connection);
  }

  List<String> chunkMessage(String message, {int pointsPerChunk = 10}) {
    final List<String> chunks = [];

    // Split the message into parts
    final parts = message.split('|');

    if (parts.length < 3) return [message]; // Not in expected format

    // Extract header parts (PREFIX, latPrefix, longPrefix)
    final header = parts.sublist(0, 3).join('|');
    final pointData = parts.sublist(3);

    // Remove empty strings and filter out non-point data
    final validPoints = pointData
        .where(
          (point) => point.isNotEmpty && point != 'E' && point.contains(','),
        )
        .toList();

    if (validPoints.isEmpty) return [message];

    // Create chunks
    for (int i = 0; i < validPoints.length; i += pointsPerChunk) {
      final endIndex = min(i + pointsPerChunk, validPoints.length);
      final chunkPoints = validPoints.sublist(i, endIndex);

      // Reconstruct chunk with header
      final chunk = '$header|${chunkPoints.join('|')}|E';
      chunks.add(chunk);
    }

    return chunks;
  }

  List<String> chunkMessageWithHeaders(
    String message, {
    int pointsPerChunk = 10,
  }) {
    final List<String> chunks = [];
    final messageId = DateTime.now().millisecondsSinceEpoch.toString();
    _currentMessageId = messageId;

    // Split the message into parts
    final parts = message.split('|');

    if (parts.length < 3) return [message];

    // Extract original header (PREFIX, latPrefix, longPrefix)
    final originalHeader = parts.sublist(0, 3).join('|');
    final pointData = parts.sublist(3);

    // Filter valid points
    final validPoints = pointData
        .where(
          (point) => point.isNotEmpty && point != 'E' && point.contains(','),
        )
        .toList();

    if (validPoints.isEmpty) return [message];

    final totalChunks = (validPoints.length / pointsPerChunk).ceil();

    // Create chunks with enhanced headers
    for (int chunkIndex = 0; chunkIndex < totalChunks; chunkIndex++) {
      final startIndex = chunkIndex * pointsPerChunk;
      final endIndex = min(startIndex + pointsPerChunk, validPoints.length);
      final chunkPoints = validPoints.sublist(startIndex, endIndex);

      // New header format: MSG_ID|TOTAL_CHUNKS|CURRENT_CHUNK|ORIGINAL_HEADER
      final header = '$originalHeader|$totalChunks|$chunkIndex';
      final chunk = '$header|${chunkPoints.join('|')}|E';

      chunks.add(chunk);
    }

    return chunks;
  }


  void _sendChunksToEsp32(List<String> chunks, BluetoothConnection connection) {
    setState(() {
      _isSending = true;
      _messageComplete = false;
      _showCompletionMessage = false;
      _messageChunks = chunks;
      _currentChunkIndex = 0;
      _totalChunks = chunks.length;
    });

    _sendNextChunkWithDelay(connection);
  }

  void _sendNextChunkWithDelay(BluetoothConnection connection) {
    if (_currentChunkIndex >= _messageChunks.length) {
      print('All chunks sent, waiting for final confirmation');
      return;
    }

    final chunk = _messageChunks[_currentChunkIndex];
    connection.writeString(chunk);

    print('Sent chunk ${_currentChunkIndex + 1}/$_totalChunks');

    // Add delay between chunks (adjust as needed)
    Future.delayed(Duration(milliseconds: 1000), () {
      if (mounted && _isSending) {
        setState(() {
          _currentChunkIndex++;
        });
        _sendNextChunkWithDelay(connection);
      }
      if(_currentChunkIndex == _totalChunks){
        _isSending = false;
      }
    });
  }


  @override
  void dispose() {
    _responseTimer?.cancel();
    _readSubscription?.cancel();
    super.dispose();
  }
}