<template>
  <div class="main-container">
    <div>
      <h1 class="main-title">은행 위치</h1>
    </div>
    <div class="search-box">
      <input
        v-model="keyword"
        placeholder="검색어를 입력하세요"
        class="search-input"
      />
      <button @click="searchPlaces" class="search-button">검색</button>
    </div>
    <div class="map-container">
      <KakaoMap
        :lat="lat"
        :lng="lng"
        @on-load-kakao-map="onLoadKakaoMap"
        class="kakao-map"
      >
        <KakaoMapMarker
          v-for="(marker, index) in markerList"
          :key="marker.key === undefined ? index : marker.key"
          :lat="marker.lat"
          :lng="marker.lng"
          :infoWindow="marker.infoWindow"
          :clickable="true"
          @onClickKakaoMapMarker="onClickMapMarker(marker)"
        />
      </KakaoMap>
      <button @click="changeLatLng" class="move-center">
        ↺ 현재 위치 검색
      </button>
      <div class="zoom-controls">
        <button class="zoom-button zoom-in" @click="zoomIn">+</button>
        <div class="divider"></div>
        <button class="zoom-button zoom-out" @click="zoomOut">-</button>
      </div>
    </div>
    <!-- <p class="message">{{ message }}</p> -->
    <hr />
    <div v-for="(marker, index) in markerList" :key="index"></div>
  </div>
</template>

<script setup>
import { KakaoMap, KakaoMapMarker } from "vue3-kakao-maps";
import { ref } from "vue";

const lat = ref(37.50131);
const lng = ref(127.0396);
const map = ref();
const latRange = [lat.value - 0.0001, lat.value];
const lngRange = [lng.value - 0.0001, lng.value];
const message = ref("");
const keyword = ref("");
const markerList = ref([]);

// 현재 위치로 이동하기
const onLoadKakaoMap = (mapRef) => {
  map.value = mapRef;
  displayLevel();
};

const changeLatLng = () => {
  if (!navigator.geolocation) {
    setAlert("위치 정보를 찾을 수 없습니다.");
  } else {
    navigator.geolocation.getCurrentPosition(
      getPositionValue,
      geolocationError
    );
  }
  lat.value = Math.random() * (latRange[1] - latRange[0]) + latRange[0];
  lng.value = Math.random() * (lngRange[1] - lngRange[0]) + lngRange[0];
};

function getPositionValue(val) {
  lat.value = val.coords.latitude;
  lng.value = val.coords.longitude;

  // 지도의 중심을 사용자의 현재 위치로 변경
  if (map.value) {
    const moveLatLon = new kakao.maps.LatLng(
      val.coords.latitude,
      val.coords.longitude
    );
    map.value.setCenter(moveLatLon);
  }
}

function geolocationError() {
  setAlert("위치 정보를 찾을 수 없습니다.");
}

function setAlert(text) {
  alert(text);
}

// 지도 확대 및 축소 구현
const zoomIn = () => {
  // 현재 지도의 레벨을 얻어옵니다
  if (map.value) {
    const level = map.value.getLevel();
    // 지도를 1레벨 내립니다 (지도가 확대됩니다)
    map.value.setLevel(level - 1);
  }
  // 지도 레벨을 표시합니다
  displayLevel();
};

const zoomOut = () => {
  // 현재 지도의 레벨을 얻어옵니다
  if (map.value) {
    const level = map.value.getLevel();
    // 지도를 1레벨 올립니다 (지도가 축소됩니다)
    map.value.setLevel(level + 1);
  }
  // 지도 레벨을 표시합니다
  displayLevel();
};

const displayLevel = () => {
  message.value = `현재 지도 레벨은 ${map.value?.getLevel()} 레벨 입니다.`;
};

const searchPlaces = () => {
  if (!keyword.value.trim()) {
    setAlert("검색어를 입력해주세요.");
    return;
  }
  const ps = new kakao.maps.services.Places();
  if (keyword.value === '은행') {
    var center = map.value.getCenter()
    lat.value = center.getLat()
    lng.value = center.getLng()
    const options = {
      location: new kakao.maps.LatLng(lat.value, lng.value),
      radius: 200,
    };
    ps.keywordSearch(keyword.value, placesSearchCB, options); // 키워드 검색 실행
  } else {
    ps.keywordSearch(keyword.value, placesSearchCB)
  }
};

// 키워드 검색 완료 시 호출되는 콜백함수 입니다
const placesSearchCB = (data, status) => {
  if (status === kakao.maps.services.Status.OK) {
    markerList.value = []
    // 검색된 장소 위치를 기준으로 지도 범위를 재설정하기위해
    // LatLngBounds 객체에 좌표를 추가합니다
    const bounds = new kakao.maps.LatLngBounds();
    console.log(data); // 검색된 결과 값들이 모여 있는 곳
    for (let marker of data) {
      const markerItem = {
        lat: marker.y,
        lng: marker.x,
        infoWindow: {
          content: `<div style="padding: 10px; background-color: white; border: 1px solid #ccc; border-radius: 5px; display: flex; flex-direction: column; align-items: flex-start;">
            <div style="font-weight: bold; margin-bottom: 5px">${marker.place_name}</div>
            <div style="display: flex">
              <div style="display: flex; flex-direction: column; align-items: flex-start">
              <div style="overflow: hidden; text-overflow: ellipsis; white-space: nowrap">${marker.address_name}</div>
              <div style="overflow: hidden; text-overflow: ellipsis; white-space: nowrap">${marker.road_address_name}</div>
              <div><a href="${marker.place_url}" target="_blank" style="color: blue">큰 지도로 보기</a></div>
            </div>
          </div>
        </div>`,
          visible: false,
        },
      };
      markerList.value.push(markerItem);
      bounds.extend(new kakao.maps.LatLng(Number(marker.y), Number(marker.x)));
    }

    // 검색된 장소 위치를 기준으로 지도 범위를 재설정합니다
    map.value?.setBounds(bounds);
  }
};

//마커 클릭 시 인포윈도우의 visible 값을 반전시킵니다
const onClickMapMarker = (markerItem) => {
  if (
    markerItem.infoWindow?.visible !== null &&
    markerItem.infoWindow?.visible !== undefined
  ) {
    markerItem.infoWindow.visible = !markerItem.infoWindow.visible;
  } else {
    markerItem.infoWindow.visible = true;
  }
};
</script>

<style scoped>
.main-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  
}
.main-title {
  font-size: 36px;
  font-weight: bold;
  color: #2c3e50;
  text-align: center;
  margin: 20px 0;
  padding: 10px;
  border-bottom: 3px solid transparent; /* 초기에는 투명한 선으로 설정 */
  position: relative; /* 부모 요소로부터 상대적 위치 설정 */
}

.main-title::after {
  content: ''; /* 가상 요소 생성 */
  position: absolute; /* 절대 위치 설정 */
  bottom: 0; /* 하단 정렬 */
  left: 50%; /* 가운데 정렬 */
  transform: translateX(-50%); /* 가운데 정렬을 위한 이동 변환 */
  border-bottom: 3px solid #3498db; /* 밑줄 색상 및 두께 설정 */
  width: 200px; /* 직접 밑줄의 길이를 조정합니다. */
}

.search-box {
  display: flex;
  justify-content: center;
  align-items: center;
  margin-bottom: 20px;
  background-color: rgba(255, 255, 255, 0.9);
  padding: 10px;
  border-radius: 10px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
  width: fit-content;
  margin: 0 auto;
}

.search-input {
  width: 200px;
  padding: 10px;
  margin-right: 10px;
  border: 1px solid #ddd;
  border-radius: 5px;
  font-size: 16px;
}

.search-button {
  background-color: #3498db;
  color: white;
  padding: 10px 20px;
  border: none;
  border-radius: 5px;
  cursor: pointer;
  font-size: 16px;
}

.search-input:focus {
  outline: none;
  border-color: #afdfff;
  box-shadow: 0 0 5px rgba(72, 174, 242, 0.5);
}

.map-container {
  position: relative;
  display: flex;
  justify-content: center;
  align-items: center;
  width: 60%;
  height: 500px;
  margin: 20px;
}

.kakao-map {
  width: 100%;
  height: 100%;
  border: 2px solid #3498db;
  border-radius: 10px;
  position: relative;
}

.map-controls {
  position: absolute;
  top: 10px;
  right: 10px; /* 오른쪽에 고정 */
  transform: translateX(-50%);
  display: flex;
  flex-direction: column;
  align-items: center;
  /* gap: 10px; */
  background-color: rgba(211, 211, 211, 0.7);
  padding: 10px;
  border-radius: 5px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
}

.move-center {
  position: absolute;
  top: 20px;
  left: 50%;
  transform: translateX(-50%);
  z-index: 1; /* 버튼을 다른 요소들 위에 표시하기 위해 z-index를 사용합니다. */
  background-color: lightslategray; /* 녹색 */
  opacity: 80%;
  color: white;
  border: none;
  cursor: pointer;
  font-size: 14px;
  padding: 6px 14px;
  border-radius: 20px; /* 둥근 모서리 */
  opacity: 80%;
  box-shadow: 0 4px 8px 0 rgba(0, 0, 0, 0.1), 0 6px 20px 0 rgba(0, 0, 0, 0.19); /* 그림자 효과 */
  transition: background-color 0.3s, transform 0.3s;
}

.move-center:hover {
  background-color: rgb(155, 168, 182); /* 호버 시 약간 더 어두운 녹색 */
  transform: translateX(-50%) scale(1.01); /* 호버 시 약간 커지는 효과 */
}

.zoom-controls {
  position: absolute;
  top: 10px;
  /* right: 10px; */
  right: 200px;
  display: flex;
  flex-direction: column;
  background-color: rgba(211, 211, 211, 0.7);
  padding: 5px;
  border-radius: 5px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
}

.zoom-button {
  background-color: rgba(211, 211, 211, 0.7);
  color: black;
  border: none;
  cursor: pointer;
  font-size: 18px;
  padding: 10px;
  border-radius: 5px;
  transition: background-color 0.3s, transform 0.3s;
}

.zoom-button:hover {
  background-color: rgba(169, 169, 169, 0.7);
}

.divider {
  width: 100%;
  height: 1px;
  background-color: #696969;
  margin: 2px 0;
}

.message {
  font-size: 16px;
  color: #2c3e50;
}
</style>
