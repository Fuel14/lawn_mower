import 'dart:async';
import 'dart:convert';
import 'dart:math';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:lawn_mower_app_ver2/data/notifiers.dart';
import 'package:geolocator/geolocator.dart';
import 'package:permission_handler/permission_handler.dart';

class OptimizedGeoData {
  final String latPrefix;
  final String longPrefix;
  final List<String> latSuffixes;
  final List<String> longSuffixes;

  OptimizedGeoData({
    required this.latPrefix,
    required this.longPrefix,
    required this.latSuffixes,
    required this.longSuffixes,
  });
}

class gpsPage extends StatefulWidget {
  const gpsPage({super.key});

  @override
  State<gpsPage> createState() => _gpsPageState();
}

class _gpsPageState extends State<gpsPage> {
  final List<LatLng> _polygonPoints = [];
  bool _isDrawing = false;
  final MapController _mapController = MapController();
  bool _isInteractive = true;
  int _selectedIndex = -1;
  late List<LatLng> _offpolygonPoints = [];
  LatLng? _currentLocation;
  bool _isLoadingLocation = false;
  StreamSubscription<Position>? _positionStreamSubscription;

  @override
  void initState() {
    super.initState();
    _checkLocationPermission();
  }

  @override
  void dispose() {
    _positionStreamSubscription?.cancel();
    super.dispose();
  }

  Future<void> _checkLocationPermission() async {
    final status = await Permission.location.status;
    if (status.isGranted) {
      _getCurrentLocation();
      _startLocationUpdates();
    } else if (status.isDenied) {
      final result = await Permission.location.request();
      if (result.isGranted) {
        _getCurrentLocation();
        _startLocationUpdates();
      }
    }
  }

  Future<void> _getCurrentLocation() async {
    setState(() => _isLoadingLocation = true);

    try {
      final position = await Geolocator.getCurrentPosition(
        desiredAccuracy: LocationAccuracy.high,
      );

      setState(() {
        _currentLocation = LatLng(position.latitude, position.longitude);
        _isLoadingLocation = false;
      });

      // Center map on current location
      _mapController.move(_currentLocation!, 15);
    } catch (e) {
      setState(() => _isLoadingLocation = false);
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text('Error getting location: $e')));
    }
  }

  void _startLocationUpdates() {
    _positionStreamSubscription = Geolocator.getPositionStream().listen((
      Position position,
    ) {
      if (mounted) {
        setState(() {
          _currentLocation = LatLng(position.latitude, position.longitude);
        });
      }
    });
  }

  void _stopLocationUpdates() {
    _positionStreamSubscription?.cancel();
    _positionStreamSubscription = null;
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Expanded(
          flex: 7,
          child: Stack(
            children: [
              FlutterMap(
                mapController: _mapController,
                options: MapOptions(
                  initialCenter: _currentLocation ?? LatLng(13.7304, 100.7749),
                  initialZoom: 15,
                  onTap: (_, latlng) {
                    if (_isDrawing) {
                      setState(() => _polygonPoints.add(latlng));
                    }
                    if (_selectedIndex != -1) {
                      setState(() {
                        _polygonPoints[_selectedIndex] = latlng;
                        _selectedIndex = -1;
                      });
                    }
                  },
                  interactionOptions: InteractionOptions(
                    flags: _isInteractive == true
                        ? InteractiveFlag.all
                        : InteractiveFlag.none,
                  ),
                ),
                children: [
                  TileLayer(
                    urlTemplate:
                        'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
                    userAgentPackageName: 'com.KMITL.lawnmowerapp',
                    tileProvider: NetworkTileProvider(
                      headers: {
                        'User-Agent':
                            'LawnMowerApp/2.0 (com.yourcompany.lawnmowerapp)',
                      },
                    ),
                    keepBuffer: 30,
                  ),
                  RichAttributionWidget(
                    attributions: [
                      TextSourceAttribution(
                        'OpenStreetMap contributors',
                        onTap: () {},
                      ),
                    ],
                  ),
                  // Current location marker
                  if (_currentLocation != null)
                    MarkerLayer(
                      markers: [
                        Marker(
                          point: _currentLocation!,
                          width: 40,
                          height: 40,
                          child: Icon(
                            Icons.my_location,
                            color: Colors.blue,
                            size: 30,
                          ),
                        ),
                      ],
                    ),
                  // Existing markers
                  MarkerLayer(
                    markers: _polygonPoints.asMap().entries.map((entry) {
                      int index = entry.key;
                      LatLng point = entry.value;
                      return Marker(
                        point: point,
                        width: 100,
                        height: 125,
                        child: GestureDetector(
                          onLongPress: () {
                            setState(() {
                              _isDrawing = false;
                              _selectedIndex = index;
                            });
                          },
                          child: Column(
                            mainAxisSize: MainAxisSize.min,
                            children: [
                              Text(
                                '${index + 1}: ${point.latitude.toStringAsFixed(6)},${point.longitude.toStringAsFixed(6)}',
                                style: TextStyle(
                                  color: Colors.black,
                                  fontWeight: FontWeight.bold,
                                  fontSize: 8,
                                ),
                              ),
                              Icon(
                                Icons.location_pin,
                                color: _selectedIndex == index
                                    ? Colors.blue
                                    : Colors.red,
                                size: 40,
                              ),
                            ],
                          ),
                        ),
                      );
                    }).toList(),
                  ),
                  PolygonLayer(
                    polygons: [
                      if (_polygonPoints.length > 2)
                        Polygon(
                          points: _polygonPoints,
                          color: Colors.green.withOpacity(0.3),
                          borderColor: Colors.green,
                          borderStrokeWidth: 2,
                        ),
                      if (_offpolygonPoints.length > 2)
                        Polygon(
                          points: _offpolygonPoints,
                          color: Colors.red.withOpacity(0.3),
                          borderColor: Colors.red,
                          borderStrokeWidth: 2,
                        ),
                    ],
                  ),
                ],
              ),
              if (_isLoadingLocation)
                Center(child: CircularProgressIndicator()),
            ],
          ),
        ),
        Expanded(
          flex: 2,
          child: Column(
            children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: [
                  FilledButton(
                    onPressed: () {
                      setState(() {
                        _isDrawing = !_isDrawing;
                        if (!_isDrawing && _polygonPoints.length < 3) {
                          _polygonPoints.clear();
                        }
                      });
                    },
                    style: FilledButton.styleFrom(
                      backgroundColor: _isDrawing == true
                          ? Colors.blue
                          : Colors.teal,
                    ),
                    child: _isDrawing == true
                        ? Text('Now Add P')
                        : Text('To Add P'),
                  ),
                  FilledButton(
                    onPressed: () {
                      setState(() {
                        _isInteractive = !_isInteractive;
                      });
                    },
                    style: FilledButton.styleFrom(
                      backgroundColor: _isInteractive == true
                          ? Colors.teal
                          : Colors.blue,
                    ),
                    child: _isInteractive == true
                        ? Text('To Lock Map')
                        : Text('Locking Map'),
                  ),
                  FilledButton(
                    onPressed: () {
                      setState(() {
                        final bluetoothMessage =
                            createOptimizedBluetoothMessage(_polygonPoints);
                        gpsDataNotifier.value = bluetoothMessage;
                      });
                    },
                    child: Text('Save debug'),
                  ),
                ],
              ),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: [
                  FilledButton(
                    onPressed: () {
                      setState(() {
                        _polygonPoints.clear();
                        _offpolygonPoints.clear();
                      });
                    },
                    child: Text('Reset list'),
                  ),
                  FilledButton(
                    onPressed: _getCurrentLocation,
                    child: Text('Get Location'),
                    style: FilledButton.styleFrom(backgroundColor: Colors.blue),
                  ),
                  FilledButton(
                    onPressed: () {
                      if (_polygonPoints.length < 3) {
                        ScaffoldMessenger.of(context).showSnackBar(
                          const SnackBar(
                            content: Text(
                              'Need at least 3 points to create a polygon',
                            ),
                          ),
                        );
                        return;
                      }
                      if (_polygonPoints.length > 2) {
                        setState(() {
                          _offpolygonPoints = generateMowingPath(
                            _polygonPoints,
                            cuttingWidth: 1.0,
                          );
                          _savePolygon();
                          final bluetoothMessage =
                              createOptimizedBluetoothMessage(
                                _offpolygonPoints,
                              );
                          gpsDataNotifier.value = bluetoothMessage;
                        });
                      }
                    },
                    child: Text('Save list & Sent'),
                  ),
                ],
              ),
            ],
          ),
        ),
      ],
    );
  }

  void _savePolygon() {
    if (_polygonPoints.length < 3) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('Need at least 3 points to create a polygon'),
        ),
      );
      return;
    }

    print('Polygon points saved: $_polygonPoints');
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text('Saved ${_polygonPoints.length} points')),
    );
  }

  List<LatLng> generateMowingPath(
    List<LatLng> polygonVertices, {
    double cuttingWidth = 1.0, // 1 meter cutting width
    double overlap = 0.2, // 20% overlap between passes
  }) {
    final List<LatLng> mowingPath = [];

    if (polygonVertices.length < 3) return mowingPath;

    // 1. Calculate polygon bounding box
    double minLat = polygonVertices[0].latitude;
    double maxLat = polygonVertices[0].latitude;
    double minLng = polygonVertices[0].longitude;
    double maxLng = polygonVertices[0].longitude;

    for (final point in polygonVertices) {
      minLat = point.latitude < minLat ? point.latitude : minLat;
      maxLat = point.latitude > maxLat ? point.latitude : maxLat;
      minLng = point.longitude < minLng ? point.longitude : minLng;
      maxLng = point.longitude > maxLng ? point.longitude : maxLng;
    }

    // 2. Calculate step sizes (adjusted for overlap)
    final double effectiveWidth = cuttingWidth * (1 - overlap);
    final double latStep = effectiveWidth / 111320; // 1° latitude ≈ 111,320m
    final double avgLat = (minLat + maxLat) / 2;
    final double lngStep = effectiveWidth / (111320 * cos(avgLat * pi / 180));

    // 3. Generate parallel mowing lines (alternating directions)
    bool reverseDirection = false;
    for (double lat = minLat; lat <= maxLat; lat += latStep) {
      // Find min/max longitude for this latitude
      double lineMinLng = maxLng;
      double lineMaxLng = minLng;

      for (double lng = minLng; lng <= maxLng; lng += lngStep) {
        final testPoint = LatLng(lat, lng);
        if (_isPointInPolygon(testPoint, polygonVertices)) {
          lineMinLng = lng < lineMinLng ? lng : lineMinLng;
          lineMaxLng = lng > lineMaxLng ? lng : lineMaxLng;
        }
      }

      // Add points in alternating directions (boustrophedon pattern)
      if (lineMinLng < lineMaxLng) {
        final List<LatLng> linePoints = [];
        double lng = reverseDirection ? lineMaxLng : lineMinLng;
        final double endLng = reverseDirection ? lineMinLng : lineMaxLng;

        linePoints.add(LatLng(lat, lng));
        linePoints.add(LatLng(lat, endLng));

        if (reverseDirection) linePoints.reversed;
        mowingPath.addAll(linePoints);
        reverseDirection = !reverseDirection;
      }
    }
    print(mowingPath);
    return mowingPath;
  }

  bool _isPointInPolygon(LatLng point, List<LatLng> polygon) {
    // Ray casting algorithm implementation
    bool inside = false;
    for (int i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
      final LatLng vertexI = polygon[i];
      final LatLng vertexJ = polygon[j];

      if (((vertexI.longitude > point.longitude) !=
              (vertexJ.longitude > point.longitude)) &&
          (point.latitude <
              (vertexJ.latitude - vertexI.latitude) *
                      (point.longitude - vertexI.longitude) /
                      (vertexJ.longitude - vertexI.longitude) +
                  vertexI.latitude)) {
        inside = !inside;
      }
    }
    return inside;
  }

  OptimizedGeoData optimizeGeoData(List<LatLng> points) {
    if (points.isEmpty) {
      return OptimizedGeoData(
        latPrefix: '',
        longPrefix: '',
        latSuffixes: [],
        longSuffixes: [],
      );
    }

    // Convert all points to strings with full precision
    final latStrings = points.map((p) => p.latitude.toString()).toList();
    final longStrings = points.map((p) => p.longitude.toString()).toList();

    // Find common prefix for latitudes
    String latPrefix = _findCommonPrefix(latStrings);
    // Find common prefix for longitudes
    String longPrefix = _findCommonPrefix(longStrings);

    // Extract suffixes (the parts after the common prefix)
    final latSuffixes = latStrings
        .map((s) => s.substring(latPrefix.length))
        .toList();
    final longSuffixes = longStrings
        .map((s) => s.substring(longPrefix.length))
        .toList();

    return OptimizedGeoData(
      latPrefix: latPrefix,
      longPrefix: longPrefix,
      latSuffixes: latSuffixes,
      longSuffixes: longSuffixes,
    );
  }

  String _findCommonPrefix(List<String> strings) {
    if (strings.isEmpty) return '';

    String prefix = strings[0];
    for (int i = 1; i < strings.length; i++) {
      while (!strings[i].startsWith(prefix)) {
        prefix = prefix.substring(0, prefix.length - 1);
        if (prefix.isEmpty) return '';
      }
    }
    return prefix;
  }

  String createOptimizedBluetoothMessage(List<LatLng> points) {
    final optimized = optimizeGeoData(points);
    print(optimized.latSuffixes.length);
    // Create compact message format:
    // "PREFIX|latPrefix|longPrefix|latSuffix1,longSuffix1|latSuffix2,longSuffix2|..."
    final buffer = StringBuffer();
    buffer.write('P|${optimized.latPrefix}|${optimized.longPrefix}|');

    for (int i = 0; i < optimized.latSuffixes.length; i++) {
      if (i > 0) buffer.write('|');
      buffer.write('${optimized.latSuffixes[i]},${optimized.longSuffixes[i]}');
    }
    return buffer.toString();
  }
}
