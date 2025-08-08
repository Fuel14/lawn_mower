import 'package:flutter/material.dart';

class HeroWidget extends StatelessWidget {
  const HeroWidget({super.key});

  @override
  Widget build(BuildContext context) {
    return Hero(
      tag: 'her1',
      child: AspectRatio(
        aspectRatio: 1920/1080,
        child: ClipRRect(
          borderRadius: BorderRadius.circular(20.0),
          child: Image.asset('assets/images/elong.jpg', fit: BoxFit.cover,),
        ),
      ),
    );
  }
}
