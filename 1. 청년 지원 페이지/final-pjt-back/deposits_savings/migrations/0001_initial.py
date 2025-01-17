# Generated by Django 4.2.8 on 2024-05-17 05:12

from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):

    initial = True

    dependencies = [
    ]

    operations = [
        migrations.CreateModel(
            name='Deposit',
            fields=[
                ('fin_co_no_fin_prdt_cd', models.CharField(max_length=100, primary_key=True, serialize=False)),
                ('fin_prdt_cd', models.CharField(max_length=100)),
                ('dcls_month', models.CharField(max_length=10)),
                ('fin_co_no', models.CharField(max_length=15)),
                ('kor_co_nm', models.CharField(max_length=100)),
                ('fin_prdt_nm', models.CharField(max_length=100)),
                ('join_way', models.TextField(null=True)),
                ('mtrt_int', models.TextField()),
                ('spcl_cnd', models.TextField()),
                ('join_deny', models.CharField(max_length=10)),
                ('join_member', models.CharField(max_length=200)),
                ('etc_note', models.TextField()),
                ('max_limit', models.CharField(max_length=100, null=True)),
                ('dcls_strt_day', models.CharField(max_length=30, null=True)),
                ('dcls_end_day', models.CharField(max_length=30, null=True)),
                ('fin_co_subm_day', models.CharField(max_length=30, null=True)),
            ],
        ),
        migrations.CreateModel(
            name='Saving',
            fields=[
                ('fin_co_no_fin_prdt_cd', models.CharField(max_length=100, primary_key=True, serialize=False)),
                ('fin_prdt_cd', models.CharField(max_length=100)),
                ('dcls_month', models.CharField(max_length=100)),
                ('fin_co_no', models.CharField(max_length=100)),
                ('kor_co_nm', models.CharField(max_length=100)),
                ('fin_prdt_nm', models.CharField(max_length=100)),
                ('join_way', models.TextField(null=True)),
                ('mtrt_int', models.TextField()),
                ('spcl_cnd', models.TextField()),
                ('join_deny', models.CharField(max_length=10)),
                ('join_member', models.CharField(max_length=200)),
                ('etc_note', models.TextField()),
                ('max_limit', models.CharField(max_length=100, null=True)),
                ('dcls_strt_day', models.CharField(max_length=30, null=True)),
                ('dcls_end_day', models.CharField(max_length=30, null=True)),
                ('fin_co_subm_day', models.CharField(max_length=30, null=True)),
            ],
        ),
        migrations.CreateModel(
            name='Savingoption',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('intr_rate_type', models.CharField(max_length=100)),
                ('intr_rate_type_nm', models.CharField(max_length=100)),
                ('rsrv_type', models.CharField(max_length=100)),
                ('rsrv_type_nm', models.CharField(max_length=100)),
                ('save_trm', models.IntegerField()),
                ('intr_rate', models.FloatField()),
                ('max_intr_rate', models.FloatField()),
                ('saving', models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, to='deposits_savings.saving')),
            ],
        ),
        migrations.CreateModel(
            name='Depositoption',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('intr_rate_type', models.CharField(max_length=10)),
                ('intr_rate_type_nm', models.CharField(max_length=10)),
                ('save_trm', models.IntegerField()),
                ('intr_rate', models.FloatField(null=True)),
                ('max_intr_rate', models.FloatField()),
                ('deposit', models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, to='deposits_savings.deposit')),
            ],
        ),
    ]
