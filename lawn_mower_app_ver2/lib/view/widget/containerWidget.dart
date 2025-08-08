import 'package:flutter/material.dart';
import 'package:lawn_mower_app_ver2/data/constants.dart';

class Containerwidget extends StatelessWidget {
  const Containerwidget({super.key,required this.title, required this.description});

  final String title;
  final String description;

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: double.infinity,
        child: Card(
          child: Padding(
            padding: EdgeInsets.all(10.0),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(title, style: KTextStyle.titleBoldText),
                Text(description, style: KTextStyle.description),
              ],
            ),
          ),
        ),
      );
  }
}
