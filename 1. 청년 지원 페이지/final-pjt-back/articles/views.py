from rest_framework.response import Response
from rest_framework.decorators import api_view
from rest_framework import status

# permission Decorators
from rest_framework.decorators import permission_classes
from rest_framework.permissions import IsAuthenticated

from django.shortcuts import get_object_or_404, get_list_or_404

from .serializers import ArticleListSerializer, ArticleSerializer
from .models import Article

from .serializers import CommentSerializer
from .models import Comment

# 게시글 생성과 조회는 로그인 한 사용자만 가능
@api_view(['GET', 'POST'])
@permission_classes([IsAuthenticated])
def article_list(request):
    if request.method == 'GET':
        # get_list_or_404: 빈 값은 유효하지 않은 결과값
        # list가 없으면, 즉 조회된 것이 아무것도 없다면 404 에러를 발생시키는 것
        # articles = get_list_or_404(Article)

        # all ORM 메서드: 빈 값/리스트도 응답으로 갈 수 있음, 즉 빈 값/리스트를 반환함
        # Article에 모든 게시글을 삭제했을 때 db에 아무것도 없음 -> all은 빈 리스트를 반환하기 때문에 에러 없이 잘 작동됨
        articles = Article.objects.all()
        serializer = ArticleListSerializer(articles, many=True)
        return Response(serializer.data)

    elif request.method == 'POST':
        serializer = ArticleSerializer(data=request.data)
        # 예외를 발생시킬 것이냐? True(응) -> 유효성 검사가 통과하지 않았을 때 예외 발생시키는 것
        if serializer.is_valid(raise_exception=True):
            # serializer.save()
            serializer.save(user=request.user)
            return Response(serializer.data, status=status.HTTP_201_CREATED)


# 디테일
@api_view(['GET'])
def article_detail(request, article_pk):
    article = get_object_or_404(Article, pk=article_pk)

    if request.method == 'GET':
        serializer = ArticleSerializer(article)
        print(serializer.data)
        return Response(serializer.data)


@api_view(['GET', 'DELETE', 'PUT'])
def article_detail(request, article_pk):
    article = get_object_or_404(Article, pk=article_pk)

    if request.method == 'GET':
        serializer = ArticleSerializer(article)
        return Response(serializer.data)
    
    elif request.method == 'DELETE':
        article.delete()

        return Response({'key':'valueeee'},status=status.HTTP_204_NO_CONTENT)
    
    elif request.method == 'PUT':
        serializer = ArticleSerializer(article, data=request.data)
        if serializer.is_valid(raise_exception=True):
            serializer.save()
            return Response(serializer.data)


@api_view(['POST'])
def comment_create(request, article_pk):
    content = request.data.get('content')
    print(content)
    article = get_object_or_404(Article, pk=article_pk)
    comment = Comment.objects.create(user=request.user, content=content, article=article)
    # print(comment)
    # return Response({'success': 'Comment created successfully', 'comment': comment.id})
    serializer = CommentSerializer(comment)  # 댓글 객체를 시리얼라이즈
    return Response(serializer.data, status=status.HTTP_201_CREATED)

@api_view(['GET'])
def comment_list(request):
    if request.method == 'GET':
        comments = Comment.objects.all()
        serializer = CommentSerializer(comments, many=True)
        return Response(serializer.data)