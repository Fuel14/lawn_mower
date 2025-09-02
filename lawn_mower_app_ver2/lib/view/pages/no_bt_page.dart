import 'package:flutter/material.dart';
import 'package:lawn_mower_app_ver2/data/constants.dart';
import 'package:lawn_mower_app_ver2/view/widget/containerWidget.dart';
import 'package:lawn_mower_app_ver2/view/widget/hero_widget.dart';

class NoBtPage extends StatelessWidget {
  const NoBtPage({super.key});

  @override
  Widget build(BuildContext context) {
    double widthscreen =MediaQuery.of(context).size.width;
    return SingleChildScrollView(
      child: Center(
        child: Padding(
          padding: EdgeInsets.all(20.0),
          child: LayoutBuilder(
            builder: (context, BoxConstraints constraints) {
              return FractionallySizedBox(
                widthFactor: widthscreen > 500 ? 0.5 : 1.0,
                child: Column(
                  children: [
                    FittedBox(
                    ),
                    SizedBox(height: 20.0),
                    HeroWidget(),
                    SizedBox(height: 20.0),
                    Card(
                      child: Text("U need to connect bluetooth device first",style: KTextStyle.titleBoldText,),
                    )
                  ],
                ),
              );
            },
          ),
        ),
      ),
    );
  }
}
