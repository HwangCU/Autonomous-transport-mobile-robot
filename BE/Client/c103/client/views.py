# client/views.py
import json
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt


@csrf_exempt
def main_dispatch_view(request):
    """
    FE로부터 받은 POST 요청의 JSON 데이터에서 action 값을 확인하고,
    action이 "login"인 경우 login 앱의 로직을 호출합니다.
    """
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        data = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON"}, status=400)

    action = data.get("action")

    if action == "login":
        # login 앱의 login_logic 함수 호출
        from login.views import login_logic

        response = login_logic(data)
        return JsonResponse(response)

    # 다른 action에 대한 처리도 추가 가능
    else:
        return JsonResponse(
            {"status": "error", "message": "Unknown action"}, status=400
        )
