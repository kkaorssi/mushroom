# Automated robot system for dewdrop selection and harvesting
## Overview
이 프로젝트는 이슬송이의 선별 및 수확 작업을 자동화하는 로봇 시스템을 개발하는 것입니다.
이 시스템은 컨베이어를 통해 공급되는 이슬송이를 처리하며, 선별 공정과 수확 공정을 포함합니다.
**YOLOv8**과 **Detectron2**, **SSD** 모델을 사용하여 이슬송이 인식을 수행하고, 모델의 성능을 비교 분석하여 최적을 모델을 적용하였습니다.

## Main Features
- 이슬송이 인식: Detectron2 모델을 사용하여 이슬송이를 인식합니다.
- 선별 공정: 발아된 이슬송이의 선별 작업을 수행하며, 썩은 이슬송이와 밀집된 이슬송이를 제거합니다.
- 수확 공정: 다 자란 이슬송이를 다치지 않게 수확합니다.
- 계층적 군집 분석: 이슬송이 간의 밀집도와 거리 분석을 통해 작업 효율성을 극대화합니다.

## System Configuration
1. 이슬송이 인식:
- Detectron2: 선행 연구를 조사하고, 연구에서 사용된 모델들을 직접 구현하여 성능을 비교 분석하였습니다. 이슬송이의 사례에서는 Detectron2가 가장 좋은 성능을 보여 채택하였습니다.
2. 선별 공정:
- 썩은 이슬송이 제거: Detectron2 모델로 썩은 이슬송이를 탐지하여 제거합니다.
- 밀집도 분석: 계층적 군집 분석을 통해 밀집도가 높은 이슬송이를 제거하여 이슬송이 간의 최소 거리를 이슬송이 지름에 맞추어 설정합니다.
3. 수확 공정:
- 거리 기반 작업: 계층적 군집 분석을 통해 인접 이슬송이와의 거리가 먼 순으로 수확하여 핑거와 이슬송이 간의 충돌을 최소화합니다.

## License
이 프로젝트는 Apache License 2.0에 따라 배포됩니다.
