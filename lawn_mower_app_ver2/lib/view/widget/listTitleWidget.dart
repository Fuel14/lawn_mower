import 'package:flutter/material.dart';
import 'package:lawn_mower_app_ver2/data/constants.dart';

class Listtitlewidget extends StatelessWidget {
  const Listtitlewidget({
    super.key,
    required this.title,
    required this.avalableStaus,
    required this.selectStaus,
  });
  final String title;
  final bool avalableStaus;
  final bool selectStaus;
  @override
  Widget build(BuildContext context) {
    return ListTile(
      leading: Icon(Icons.bluetooth),
      tileColor: Colors.teal,
      title: Text(
        title,
        style: KTextStyle.titleBoldText,
      ),
      onTap: () {
        
      },
    );
  }
}
