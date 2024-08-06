# pif-betaflight

이 프로젝트는 PIF(Platform-independent framework)을 실전에 적용하여 성능을 검증하기 위함이다.

baseflight로 pif의 성능을 검증하였는데 baseflight-configurator가 더 이상 chrome-app에서 지원하지 않아 cleanflight V1.12.1로 옮기기로 결정하였다.
cleanflight V1.12.1는 baseflight와 가장 유사하고 cleanflight-configurator의 최신 버전과 호환이 되어 결정하게 되었다.
하지만 보유하고 있는 NAZE32 board 지원과 설정 백업 기능과 여러가지 문제점으로 더 이상 진행이 어려워 중단했다.
이후 새로운 SPEEDYBEE F405 V3를 구입하였고 이 보드에 적합한 betaflight의 V4.3.2로 옮기기로 결정하였다.
이를 기반으로 PIF를 적용하여 성능을 검증할 예정이다.

PIF를 포함한 이 프로그램은 충분한 검증이 된 프로그램이 아니기에 실 생활에 적용하기 부적합하고 학습용이나 연구용으로 사용되기를 바랍니다.
이 프로그램으로 인한 그 어떤 문제도 제작자에게 책임이 없으며 보증도 하지 않습니다.

---

# 감사의 글

이 프로젝트를 진행하기 위해서는 드론의 다양한 기능을 알아야 하지만 이는 불가능하다. 그래서 아래와 같은 기능에 대해 이미 개발한 프로그램들을 참고하여 작성하였다. 개발하고 참여한 이들에게 감사의 뜻을 표한다.

betaflight : https://github.com/betaflight
