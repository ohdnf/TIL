import art

from django.shortcuts import render
from django.http import JsonResponse

# Create your views here.
def ping(request):
    return render(request, 'artii/ping.html')

def pong(request):
    art_text = art.text2art(request.GET.get('inputText'))
    data = {
        'success': True,
        'content': art_text,
    }
    return JsonResponse(data)