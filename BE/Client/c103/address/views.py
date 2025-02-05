from django.shortcuts import render

# Create your views here.
import requests
import json
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt


@csrf_exempt
def get_address(request):
    """사용자로부터 주소 키워드를 받아 행안부 도로명주소 API를 호출하고 결과 반환"""
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST requests are allowed"}, status=405
        )

    try:
        # 클라이언트에서 보낸 데이터 파싱
        data = json.loads(request.body)
        keyword = data.get("address")

        if not keyword:
            return JsonResponse(
                {"status": "error", "message": "Address keyword is required"},
                status=400,
            )

        # 행안부 도로명주소 API 요청 URL
        API_URL = "https://business.juso.go.kr/addrlink/addrLinkApi.do"
        params = {
            "currentPage": "1",
            "countPerPage": "10",
            "keyword": keyword,
            "confmKey": "devU01TX0FVVEgyMDI1MDIwNTE0MzkyOTExNTQ0OTI=",  # 실제 API 키로 변경해야 함
            "resultType": "json",
        }

        # API 호출
        response = requests.get(API_URL, params=params)
        api_result = response.json()

        # juso 리스트에서 roadAddr 값만 추출
        juso_list = api_result.get("results", {}).get("juso", [])
        road_addresses = [juso.get("roadAddr") for juso in juso_list]

        return JsonResponse({"status": "success", "road_addresses": road_addresses})

    except json.JSONDecodeError:
        return JsonResponse(
            {"status": "error", "message": "Invalid JSON format"}, status=400
        )

    except Exception as e:
        return JsonResponse(
            {"status": "error", "message": f"Internal server error: {e}"}, status=500
        )
